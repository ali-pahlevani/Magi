#!/usr/bin/env python3
"""Rebuild the Rubicon heightmap so a wheel can actually roll on it.

WHY THIS EXISTS
---------------
Upstream ships `materials/textures/Heightmap.png` as an **8-bit** greyscale
image stretched over 5 m of relief. One grey level is therefore

    5.0 / 255 = 19.6 mm

and the terrain is not a surface but a staircase: flat plateaus separated by
~2 cm risers, one every 7.3 cm cell. Measured on the original asset the mean
cell-to-cell step is 15.6 mm, i.e. essentially one quantisation level
everywhere -- the relief the author drew is smaller than the format's own
resolution over most of the map.

That is fatal for an 86 mm wheel. Mounting a step of height h with a wheel of
radius r needs a tractive force of

    F/N = sqrt(2rh - h^2) / (r - h)

which at h = 19.6 mm is **0.82**, against a friction coefficient of 1.0. Every
single cell boundary is a near-stall obstacle, and each one that is cleared
delivers an impulse into the body. That is both halves of the complaint at
once: the robot bogs down, and the impulses roll it over.

WHAT THIS DOES
--------------
1. **De-quantise.** The true surface is known to lie within +/- half a grey
   level of each sample, so the staircase is removed by smoothing under that
   constraint: repeated Laplacian relaxation, clamped back into
   [h - lvl/2, h + lvl/2] after every pass. The result is the smoothest
   surface that is still consistent with the authored image -- no sample moves
   by more than 9.8 mm, so every rock, tree and structure placed against the
   original terrain stays where it was put.

2. **Refine.** Bicubic upsample 513 -> 1025 (2^n+1, as Gazebo requires), which
   halves the facet size from 7.3 to 3.7 cm. The wheel then spans several
   facets instead of landing on one at a time.

3. **Write 16 bit.** 5.0 / 65535 = 0.076 mm per level, so the smoothed surface
   survives being written back out. Writing it 8-bit again would re-create the
   staircase the first step just removed.

The peak is pinned at full scale on purpose: `gz::common::ImageHeightmap`
normalises by the maximum pixel value present, so an image whose brightest
pixel is not full white gets stretched back up to the SDF <size> z anyway.
Pinning it makes that a no-op and keeps absolute heights identical.

Measured effect on the asset (cell-to-cell step is what the wheel climbs):

    original 8-bit  513   step mean 15.6 mm   slope p50 15.0 deg
    rebuilt 16-bit 1025   step mean  6.7 mm   slope p50 11.6 deg

The slope figures fall without the terrain changing shape: most of the p50 was
the staircase, not the hill.

USAGE
-----
    rebuild_heightmap.py <model dir>        # rewrites model.sdf to use it

Idempotent: rerunning it regenerates the PNG from the original 8-bit source
and leaves model.sdf alone if it already points at the rebuilt file.
"""

import argparse
import math
import os
import re
import sys

import numpy as np
from PIL import Image

SOURCE = "materials/textures/Heightmap.png"
OUTPUT = "materials/textures/Heightmap_smooth.png"

# Terrain vertical extent, from the <heightmap><size> z in model.sdf. Only used
# to report the quantisation step in metres; the maths is done in grey levels.
RELIEF_M = 5.0


def dequantise(h, level, iters=400):
    """Smoothest surface within +/- half a quantisation level of `h`.

    Laplacian relaxation with the constraint re-applied every pass. The
    constraint is what makes this a de-quantisation rather than a blur: it
    cannot move the surface further than the information the 8-bit image
    actually lost, so macro terrain, and anything placed on it, is untouched.
    """
    lo, hi = h - 0.5 * level, h + 0.5 * level
    x = h.copy()
    for _ in range(iters):
        k = x.copy()
        k[1:-1, 1:-1] = (x[:-2, 1:-1] + x[2:, 1:-1]
                         + x[1:-1, :-2] + x[1:-1, 2:]) / 4.0
        x = np.clip(k, lo, hi)
    return x


def upsample(h, size):
    """Bicubic resample to `size` x `size`, which must be 2^n + 1."""
    if size == h.shape[0]:
        return h
    return np.asarray(
        Image.fromarray(h.astype(np.float32), mode="F")
             .resize((size, size), Image.BICUBIC)
    ).astype(np.float64)


def report(h, cell, label):
    gy, gx = np.gradient(h, cell)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    step = np.abs(np.diff(h, axis=0))
    print(f"  {label:<22} {h.shape[0]:>5} px  cell {cell*100:5.2f} cm   "
          f"step mean {step.mean()*1000:5.1f} mm p99 {np.percentile(step,99)*1000:5.1f} mm   "
          f"slope p50 {np.percentile(slope,50):4.1f} p90 {np.percentile(slope,90):4.1f} "
          f"max {slope.max():4.1f} deg")


def patch_sdf(sdf_path, source, output):
    """Point both the collision and the visual heightmap at the rebuilt PNG."""
    text = open(sdf_path).read()
    if f"<uri>{output}</uri>" in text:
        print("  model.sdf already points at the rebuilt heightmap")
        return
    n = text.count(f"<uri>{source}</uri>")
    if n != 2:
        sys.exit(f"  unexpected model.sdf layout: {n} heightmap <uri> tags, expected 2")
    open(sdf_path, "w").write(
        text.replace(f"<uri>{source}</uri>", f"<uri>{output}</uri>"))
    print(f"  model.sdf now uses {output} for collision and visual")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model_dir", help="the Rubicon model directory")
    ap.add_argument("--size", type=int, default=1025,
                    help="output resolution, must be 2^n + 1 (default 1025)")
    ap.add_argument("--extent", type=float, default=37.5,
                    help="terrain span in metres, for the reported statistics")
    args = ap.parse_args()

    if not math.log2(args.size - 1).is_integer():
        sys.exit(f"--size must be 2^n + 1, got {args.size}")

    src = os.path.join(args.model_dir, SOURCE)
    dst = os.path.join(args.model_dir, OUTPUT)
    img = Image.open(src)
    if img.mode != "L":
        sys.exit(f"{src}: expected an 8-bit greyscale source, got mode {img.mode}")

    a = np.asarray(img).astype(np.float64) / 255.0      # 0..1
    level = 1.0 / 255.0
    n = a.shape[0]
    cell = args.extent / (n - 1)
    print(f"Rebuilding {src}")
    print(f"  8-bit source: one grey level is "
          f"{RELIEF_M/255*1000:.1f} mm of relief")
    report(a * RELIEF_M, cell, "original")

    d = dequantise(a, level)
    print(f"  de-quantised, max sample moved "
          f"{np.abs(d - a).max() * RELIEF_M * 1000:.1f} mm "
          f"(bound is {RELIEF_M/255/2*1000:.1f} mm)")

    u = np.clip(upsample(d, args.size), 0.0, 1.0)
    report(u * RELIEF_M, args.extent / (args.size - 1), "rebuilt")

    # Pin the peak at full scale: ImageHeightmap normalises by the brightest
    # pixel, so anything less would be stretched back up and shift every height.
    u = u / u.max()
    Image.fromarray(np.round(u * 65535.0).astype(np.uint16), mode="I;16").save(dst)
    print(f"  wrote {dst} ({args.size}x{args.size}, 16-bit, "
          f"{os.path.getsize(dst)/1e6:.1f} MB)")

    patch_sdf(os.path.join(args.model_dir, "model.sdf"), SOURCE, OUTPUT)


if __name__ == "__main__":
    main()
