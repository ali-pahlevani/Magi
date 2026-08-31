#!/usr/bin/env bash
#
# Fetch the Rubicon world from Gazebo Fuel into magi_gazebo/models and apply the
# terrain friction patch. Run this only to re-create or update the vendored
# copy -- the model is committed to the repo, so normal use needs no network.
#
#   src/magi_gazebo/scripts/fetch_rubicon.sh
#
# Keep this directory to the Fuel archive's own contents. magi_gazebo installs
# models/ and ament_cmake_symlink_install globs it with FILE GLOB_RECURSE, which
# follows symlinks (CMP0009 is unset inside the generated cmake_install.cmake,
# and a cmake_policy call in CMakeLists.txt does not reach it). A stray colcon
# build/ install/ log/ left in here by running `colcon build` from this
# directory puts log/latest -> latest_build in the glob's path and every
# --symlink-install build then prints a CMP0009 warning.
#
# Two patches are applied on top of the Fuel archive, and both are load-bearing:
#
#   friction   The upstream model declares the terrain heightmap collision with
#              no <surface> block at all, so it falls back to Gazebo's default
#              contact friction. Measured in this workspace, driving the Go2W
#              across the terrain went from 16% to 28% of the commanded speed
#              once an explicit friction surface was added. 1.5 is a reasonable
#              coefficient for rubber on rock.
#
#   heightmap  The shipped Heightmap.png is 8-bit over 5 m of relief, so one
#              grey level is 19.6 mm and the terrain is a staircase rather than
#              a surface -- see rebuild_heightmap.py, which regenerates it as a
#              de-quantised 16-bit 1025x1025 map and repoints model.sdf at it.
#              Worth 1.52 -> 2.18 m of ground covered per 8 s run over the
#              standard course.

set -euo pipefail

URL="https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rubicon.zip"
DEST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/models/Rubicon"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading Rubicon (~187 MB) ..."
# --retry/-C are load-bearing: a plain curl of this URL truncated at 128 MB.
curl -fL --retry 5 --retry-all-errors -C - -o "$TMP/Rubicon.zip" "$URL"

echo "Verifying archive ..."
unzip -tq "$TMP/Rubicon.zip"

echo "Extracting to $DEST ..."
rm -rf "$DEST"
mkdir -p "$DEST"
unzip -q "$TMP/Rubicon.zip" -d "$DEST"

echo "Patching heightmap collision friction ..."
python3 - "$DEST/model.sdf" <<'PY'
import sys

path = sys.argv[1]
text = open(path).read()

anchor = """          </heightmap>
        </geometry>
      </collision>"""
patched = """          </heightmap>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>"""

if "<mu>1.5</mu>" in text:
    print("  already patched")
elif text.count(anchor) == 1:
    open(path, "w").write(text.replace(anchor, patched))
    print("  friction surface added")
else:
    sys.exit(f"  unexpected model.sdf layout: found {text.count(anchor)} anchors")
PY

echo "Rebuilding the heightmap ..."
python3 "$(dirname "${BASH_SOURCE[0]}")/rebuild_heightmap.py" "$DEST"

echo "Done. Rebuild magi_gazebo to pick the model up."
