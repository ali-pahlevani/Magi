#!/usr/bin/env python3
"""Normalise COLLADA primitives that share a single index offset.

Why this exists
---------------
`mid-360.dae` (the Livox MID-360 mesh) declares every <polylist> like this:

    <input semantic="VERTEX" offset="0" .../>
    <input semantic="NORMAL" offset="0" .../>

Sharing offset 0 is legal COLLADA -- one index feeds both arrays, so <p> holds
exactly one index per vertex. gz-common5's ColladaLoader instead strides <p> by
the *number of inputs*, so it tries to read twice as many indices as the file
contains, walks off the end of the array and segfaults in parseInt. That takes
down the whole Gazebo GUI the moment the robot is spawned.

The fix is to rewrite the primitive into the form the loader expects: give each
input its own offset and expand <p> by repeating each index once per input.
That is numerically identical, because a shared offset means the same index was
already being used for every semantic.

Usage:  fix_collada_offsets.py <file.dae> [...]
"""
import sys
import xml.etree.ElementTree as ET

NS = "http://www.collada.org/2005/11/COLLADASchema"
PRIMITIVES = ("polylist", "triangles", "polygons", "trifans", "tristrips", "lines")


def fix_file(path):
    ET.register_namespace("", NS)
    tree = ET.parse(path)
    root = tree.getroot()

    fixed = 0
    for prim_name in PRIMITIVES:
        for prim in root.iter(f"{{{NS}}}{prim_name}"):
            inputs = prim.findall(f"{{{NS}}}input")
            if len(inputs) < 2:
                continue
            offsets = [int(i.get("offset", 0)) for i in inputs]
            if len(set(offsets)) != 1:
                continue  # already has distinct offsets

            p = prim.find(f"{{{NS}}}p")
            if p is None or not p.text:
                continue

            indices = p.text.split()
            n = len(inputs)
            # one shared index -> n copies, one per input
            p.text = " ".join(idx for value in indices for idx in (value,) * n)
            for slot, inp in enumerate(inputs):
                inp.set("offset", str(slot))
            fixed += 1

    if fixed:
        tree.write(path, xml_declaration=True, encoding="utf-8")
    print(f"{path}: {fixed} primitive(s) rewritten")
    return fixed


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    total = sum(fix_file(p) for p in sys.argv[1:])
    print(f"total rewritten: {total}")
