#!/usr/bin/env python3
"""Measure how much driving the robot survives, with and without balance.

The point of this tool is to make the balance controller falsifiable. It drives
a fixed profile at a fixed speed, watches the ground-truth attitude, and
reports whether the robot ended the run upright -- then repeats at higher
speeds until it does not. The result is a rollover threshold in m/s, which is a
single number that can be compared before and after a change.

    # sweep, balance active (the default)
    ros2 run magi_control magi_stability_test.py --world rubicon \
        --reset-pose 3.0,-0.5,1.85 --speeds 0.3,0.5,0.7,0.9 --turn 0.3

    # same sweep with the feedback frozen, i.e. the old fixed stance
    ros2 run magi_control magi_stability_test.py --world rubicon \
        --reset-pose 3.0,-0.5,1.85 --speeds 0.3,0.5,0.7,0.9 --turn 0.3 --no-balance

Reported per run:
    result     UPRIGHT or ROLLED, from ground-truth roll/pitch
    |roll|max  worst absolute roll seen, degrees
    roll rms   how much the body was moving, degrees -- the disturbance
               rejection number. Lower is a body better isolated from terrain.
    margin     mean and minimum CoP margin, metres. Negative means the CoP left
               the support polygon, which is the tipping condition itself.
    track      mean effective track, showing whether the widen lever engaged

A single run on Rubicon is noise. The terrain is a quantised heightmap and
where the wheels happen to land dominates any one trial, so --reps defaults to
3 and the summary reports the fraction of runs survived rather than a verdict
from one.
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
from std_msgs.msg import Bool, Float64

MODEL = "magi_go2w"
CMD_TOPIC = "/wheel_controller/cmd_vel_unstamped"
ROLLED = math.radians(50.0)      # past this the robot is not coming back


def gz_pose():
    """(x, y, z, roll, pitch, yaw) of the model straight from the server."""
    for _ in range(6):
        out = subprocess.run(["gz", "model", "-m", MODEL, "-p"],
                             capture_output=True, text=True, timeout=20).stdout
        out = re.sub(r"\x1b\[[0-9;]*m", "", out)
        if "- Pose" in out:
            break
        time.sleep(1.0)
    else:
        raise RuntimeError("gz model returned no pose after 6 attempts")

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
    return (*rows[0], *rows[1])


def reset_pose(world, xyz, settle):
    x, y, z = xyz
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000",
         "--req", f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}},'
                  ' orientation: {w: 1}'],
        capture_output=True, timeout=15)
    time.sleep(settle)


class Driver(Node):
    def __init__(self):
        super().__init__("magi_stability_test")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.enable = self.create_publisher(Bool, "/magi/balance_enable", 10)
        self.msg = Twist()
        self.create_timer(0.02, lambda: self.pub.publish(self.msg))

        self.margins = []
        self.tracks = []
        self.create_subscription(Float64, "/magi/stability_margin",
                                 lambda m: self.margins.append(m.data), 10)
        self.create_subscription(Float64, "/magi/stance_width",
                                 lambda m: self.tracks.append(m.data), 10)

    def drive(self, lin, ang):
        m = Twist()
        m.linear.x, m.angular.z = lin, ang
        self.msg = m

    def stop(self):
        self.msg = Twist()


def one_run(node, lin, ang, duration):
    node.margins.clear()
    node.tracks.clear()

    rolls, pitches = [], []
    x0, y0 = gz_pose()[:2]
    node.drive(lin, ang)

    end = time.time() + duration
    rolled = False
    while time.time() < end:
        time.sleep(0.15)
        _, _, _, r, p, _ = gz_pose()
        rolls.append(r)
        pitches.append(p)
        if abs(r) > ROLLED or abs(p) > ROLLED:
            rolled = True
            break

    node.stop()
    time.sleep(1.5)
    x1, y1 = gz_pose()[:2]

    def rms(v):
        return math.sqrt(sum(a * a for a in v) / len(v)) if v else 0.0

    return {
        "rolled": rolled,
        "roll_max": math.degrees(max(abs(r) for r in rolls)) if rolls else 0.0,
        "pitch_max": math.degrees(max(abs(p) for p in pitches)) if pitches else 0.0,
        "roll_rms": math.degrees(rms(rolls)),
        "pitch_rms": math.degrees(rms(pitches)),
        "distance": math.hypot(x1 - x0, y1 - y0),
        "margin_mean": statistics.fmean(node.margins) if node.margins else float("nan"),
        "margin_min": min(node.margins) if node.margins else float("nan"),
        "track": statistics.fmean(node.tracks) if node.tracks else float("nan"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9",
                    help="comma-separated linear speeds to sweep, m/s")
    ap.add_argument("--turn", type=float, default=0.3, help="yaw rate, rad/s")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds per run")
    ap.add_argument("--reps", type=int, default=3,
                    help="repeats per speed; one run on Rubicon is noise")
    ap.add_argument("--world", default=None, help="world name for set_pose")
    ap.add_argument("--reset-pose", default=None, help="x,y,z to reset to")
    ap.add_argument("--settle", type=float, default=8.0,
                    help="seconds to let the stance recover after a reset")
    ap.add_argument("--no-balance", action="store_true",
                    help="freeze the balance feedback: fixed stance, for A/B")
    args = ap.parse_args()

    speeds = [float(s) for s in args.speeds.split(",")]
    xyz = [float(v) for v in args.reset_pose.split(",")] if args.reset_pose else None
    if xyz and not args.world:
        sys.exit("--reset-pose needs --world")

    rclpy.init()
    node = Driver()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)

    mode = "FIXED STANCE (feedback frozen)" if args.no_balance else "BALANCE ACTIVE"
    for _ in range(5):
        node.enable.publish(Bool(data=not args.no_balance))
        time.sleep(0.2)

    print(f"\n{mode}   turn {args.turn} rad/s, {args.duration:.0f} s per run, "
          f"{args.reps} reps\n")
    header = (f"{'speed':>6}{'result':>16}{'|roll|max':>11}{'roll rms':>10}"
              f"{'|pitch|max':>11}{'dist':>8}{'margin':>9}{'min':>8}{'track':>8}")
    print(header)
    print("-" * len(header))

    summary = []
    for speed in speeds:
        survived = 0
        rows = []
        for _ in range(args.reps):
            if xyz:
                reset_pose(args.world, xyz, args.settle)
            r = one_run(node, speed, args.turn, args.duration)
            rows.append(r)
            survived += 0 if r["rolled"] else 1
            print(f"{speed:>6.2f}{'ROLLED' if r['rolled'] else 'upright':>16}"
                  f"{r['roll_max']:>11.1f}{r['roll_rms']:>10.1f}"
                  f"{r['pitch_max']:>11.1f}{r['distance']:>8.2f}"
                  f"{r['margin_mean']:>9.3f}{r['margin_min']:>8.3f}"
                  f"{r['track']:>8.3f}")
        summary.append((speed, survived, args.reps, rows))
        print("-" * len(header))

    print(f"\n{mode} -- survival\n")
    print(f"{'speed':>6}{'survived':>12}{'roll rms':>10}{'min margin':>12}")
    for speed, survived, reps, rows in summary:
        rms = statistics.fmean(r["roll_rms"] for r in rows)
        mm = min(r["margin_min"] for r in rows
                 if not math.isnan(r["margin_min"])) if rows else float("nan")
        print(f"{speed:>6.2f}{f'{survived}/{reps}':>12}{rms:>10.1f}{mm:>12.3f}")

    node.stop()
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
