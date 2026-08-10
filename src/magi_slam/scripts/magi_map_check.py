#!/usr/bin/env python3
"""Check that the SLAM map is genuinely 3D, rather than 2D wearing a cloud.

"3D SLAM" is easy to claim and easy to get wrong. A pipeline can publish a
PointCloud2, register scans in 3-DoF, and project everything onto one plane --
the topic types all look right and the map is still flat. This checks the three
places the difference actually shows up:

  /cloud_map    the 3D map. Its vertical SPREAD is the test. A cloud of points
                that all sit within a few centimetres of one height is a 2D map
                that happens to be stored as points.

  /map          the 2D occupancy grid, projected from the same graph. This is
                what Nav2 consumes. Reported for size and coverage, not as
                evidence of anything 3D.

  map -> odom   the SLAM correction. If z, roll and pitch are all identically
                zero, registration is running in 3-DoF (Reg/Force3DoF true) and
                the map cannot represent slopes no matter what it publishes.

    ros2 run magi_slam magi_map_check.py
"""

import math
import struct
import sys
import threading
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

# Below this much vertical spread the "3D" map is not describing any structure
# a 2D grid could not have described.
FLAT_LIMIT = 0.5
# Rotation/translation smaller than this is indistinguishable from exactly zero.
PLANAR_LIMIT = 1e-6

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def read_z(msg, stride=1):
    """z of every `stride`-th point, skipping non-finite values."""
    offset = next((f.offset for f in msg.fields if f.name == "z"), None)
    if offset is None:
        return []
    out = []
    step = msg.point_step * stride
    for base in range(0, len(msg.data) - msg.point_step + 1, step):
        (z,) = struct.unpack_from("<f", msg.data, base + offset)
        if math.isfinite(z):
            out.append(z)
    return out


class Checker(Node):
    def __init__(self):
        super().__init__("magi_map_check")
        self.cloud = None
        self.grid = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointCloud2, "/cloud_map",
                                 lambda m: setattr(self, "cloud", m), LATCHED)
        self.create_subscription(OccupancyGrid, "/map",
                                 lambda m: setattr(self, "grid", m), LATCHED)


def main():
    rclpy.init()
    node = Checker()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + 20
    while time.time() < deadline and (node.cloud is None or node.grid is None):
        time.sleep(0.5)

    verdicts = []

    print("\n=== 3D map  (/cloud_map) ===")
    if node.cloud is None:
        print("  nothing published; is rtabmap running and has it seen a scan?")
        verdicts.append(False)
    else:
        c = node.cloud
        n = c.width * c.height
        # Big maps do not need every point to measure a spread.
        zs = read_z(c, stride=max(1, n // 40000))
        print(f"  frame     : {c.header.frame_id}")
        print(f"  points    : {n}")
        print(f"  fields    : {[f.name for f in c.fields]}")
        if zs:
            lo, hi = min(zs), max(zs)
            mean = sum(zs) / len(zs)
            sd = math.sqrt(sum((z - mean) ** 2 for z in zs) / len(zs))
            print(f"  z range   : {lo:+.2f} .. {hi:+.2f} m  "
                  f"(spread {hi - lo:.2f}, sd {sd:.2f}, from {len(zs)} sampled)")
            ok = (hi - lo) > FLAT_LIMIT
            print("  -> 3D: real vertical structure" if ok else
                  "  -> FLAT: this is a 2D map wearing a cloud")
            verdicts.append(ok)
        else:
            print("  no finite z values")
            verdicts.append(False)

    print("\n=== 2D map  (/map) ===")
    if node.grid is None:
        print("  nothing published")
    else:
        g = node.grid
        res = g.info.resolution
        known = sum(1 for v in g.data if v >= 0)
        occupied = sum(1 for v in g.data if v >= 50)
        cells = max(len(g.data), 1)
        print(f"  frame     : {g.header.frame_id}")
        print(f"  size      : {g.info.width} x {g.info.height} cells @ "
              f"{res:.3f} m  ({g.info.width * res:.1f} x {g.info.height * res:.1f} m)")
        print(f"  known     : {known} cells ({100.0 * known / cells:.1f}%)")
        print(f"  occupied  : {occupied} cells")

    print("\n=== SLAM correction  (map -> odom) ===")
    try:
        t = node.buffer.lookup_transform("map", "odom", rclpy.time.Time()).transform
        q = t.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        print(f"  xyz       : {t.translation.x:+.4f} {t.translation.y:+.4f} "
              f"{t.translation.z:+.4f}")
        print(f"  roll/pitch: {math.degrees(roll):+.3f} / {math.degrees(pitch):+.3f} deg")
        planar = (abs(t.translation.z) < PLANAR_LIMIT
                  and abs(roll) < PLANAR_LIMIT and abs(pitch) < PLANAR_LIMIT)
        print("  -> PLANAR: correction is 3-DoF only" if planar else
              "  -> 6-DoF: z / roll / pitch are being corrected")
        verdicts.append(not planar)
    except Exception as exc:
        print(f"  map -> odom not available ({exc})")
        verdicts.append(False)

    print()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(verdicts) else 1)


if __name__ == "__main__":
    main()
