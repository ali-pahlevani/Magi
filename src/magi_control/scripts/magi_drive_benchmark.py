#!/usr/bin/env python3
"""Measure what the robot actually does against what it was told to do.

Commands a constant twist, waits for steady state, then compares Gazebo
ground-truth motion with the command. Reads the simulator's own pose rather
than odometry, because wheel odometry over-reads yaw badly on a skid-steer.

    ros2 run magi_control magi_drive_benchmark.py 1.0 0.0 3.5 --reps 8 \\
        --reset-world rubicon --reset-pose 3.0,-0.5,1.85

ALWAYS USE --reps ON TERRAIN. A single run on rubicon.sdf is close to
worthless: run-to-run spread is ~10 points of standard deviation because the
robot veers onto different ground each time, and early single-run measurements
in this project produced everything from 15% to 79% for identical
configurations. Flat ground, by contrast, repeats to sd 0.2.

Reference figures, 8 reps of 3.5 s at 1.0 m/s from a fixed reset pose:

    flat.sdf                 88.8%  (sd 0.2)
    rubicon.sdf, basin       68.6%  (sd 10.2)
"""

import argparse
import math
import re
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MODEL = "magi_go2w"
CMD_TOPIC = "/wheel_controller/cmd_vel_unstamped"


def ground_truth():
    """(x, y, z, yaw) of the model straight from the Gazebo server."""
    for _ in range(6):
        out = subprocess.run(
            ["gz", "model", "-m", MODEL, "-p"],
            capture_output=True, text=True, timeout=20,
        ).stdout
        out = re.sub(r"\x1b\[[0-9;]*m", "", out)
        if "- Pose" in out:
            break
        time.sleep(1.0)
    else:
        raise RuntimeError("gz model returned no pose after 6 attempts")

    # The two bracketed rows after the "- Pose" header are XYZ then RPY.
    # Anchoring here avoids matching the entity id in "Model: [263]" or the
    # bracketed fragments inside sdformat warnings.
    lines = out.splitlines()
    start = next(i for i, ln in enumerate(lines) if "- Pose" in ln)
    rows = []
    for ln in lines[start + 1:]:
        m = re.match(r"\s*\[([-\d.eE+ ]+)\]\s*$", ln)
        if not m:
            break
        rows.append([float(v) for v in m.group(1).split()])
        if len(rows) == 2:
            break
    return rows[0][0], rows[0][1], rows[0][2], rows[1][2]


def reset_pose(world, xyz):
    x, y, z = xyz
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000",
         "--req", f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}},'
                  ' orientation: {w: 1}'],
        capture_output=True, timeout=15,
    )
    time.sleep(6.0)   # let the stance settle before commanding motion


class Driver(Node):
    def __init__(self):
        super().__init__("magi_drive_benchmark")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.msg = Twist()
        self.create_timer(0.02, lambda: self.pub.publish(self.msg))


def one_run(node, lin, ang, window):
    node.msg.linear.x = lin
    node.msg.angular.z = ang
    time.sleep(2.5)                      # discovery + acceleration ramp

    x0, y0, z0, yaw0 = ground_truth()
    t0 = time.time()

    # Sample often enough that yaw never advances more than pi between reads
    # and accumulate the unwrapped total; a single start/end pair aliases on a
    # fast spin and reports a small or negative rate.
    total_yaw, prev_yaw, z_min = 0.0, yaw0, z0
    while time.time() - t0 < window:
        time.sleep(0.4)
        x1, y1, z1, yaw = ground_truth()
        total_yaw += math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
        prev_yaw = yaw
        z_min = min(z_min, z1)
    dt = time.time() - t0

    node.msg = Twist()                   # stop
    time.sleep(1.0)

    dist = math.hypot(x1 - x0, y1 - y0)
    return {
        "speed": dist / dt,
        "yaw_rate": total_yaw / dt,
        "lin_eff": 100.0 * (dist / dt) / abs(lin) if abs(lin) > 1e-6 else None,
        "ang_eff": 100.0 * (total_yaw / dt) / ang if abs(ang) > 1e-6 else None,
        "z_drop": z0 - z_min,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("linear", type=float)
    ap.add_argument("angular", type=float)
    ap.add_argument("duration", type=float)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--reset-world", default=None,
                    help="World name for the set_pose service, e.g. rubicon")
    ap.add_argument("--reset-pose", default=None, help="x,y,z to reset to")
    args = ap.parse_args()

    xyz = None
    if args.reset_pose:
        xyz = [float(v) for v in args.reset_pose.split(",")]
        if not args.reset_world:
            sys.exit("--reset-pose needs --reset-world")

    rclpy.init()
    node = Driver()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    runs = []
    for i in range(args.reps):
        if xyz:
            reset_pose(args.reset_world, xyz)
        r = one_run(node, args.linear, args.angular, args.duration)
        runs.append(r)
        label = f"  run {i+1}: {r['speed']:.3f} m/s"
        if r["lin_eff"] is not None:
            label += f"  ({r['lin_eff']:.1f}% linear)"
        if r["ang_eff"] is not None:
            label += f"  {r['yaw_rate']:.3f} rad/s ({r['ang_eff']:.1f}% angular)"
        print(label)

    node.destroy_node()
    rclpy.shutdown()

    print(f"\ncommanded: linear {args.linear} m/s, angular {args.angular} rad/s, "
          f"{args.duration} s x {args.reps}")
    for key, name in (("lin_eff", "linear efficiency"),
                      ("ang_eff", "angular efficiency")):
        vals = [r[key] for r in runs if r[key] is not None]
        if not vals:
            continue
        sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
        print(f"  {name:<20} mean {statistics.mean(vals):5.1f}%   sd {sd:4.1f}"
              f"   min {min(vals):5.1f}  max {max(vals):5.1f}")
    if args.reps == 1:
        print("  NOTE: single run. On terrain use --reps 8 or the number is noise.")


if __name__ == "__main__":
    main()
