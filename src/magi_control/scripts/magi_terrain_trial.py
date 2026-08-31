#!/usr/bin/env python3
"""Drive a fixed course across the terrain and report what actually happened.

WHY THIS AND NOT magi_drive_benchmark.py
----------------------------------------
The drive benchmark answers "what fraction of the commanded speed does the
robot make", from one spot, and it reads ground truth by shelling out to
`gz model` a few times a second. That is the right tool for tuning a controller
against a repeatable baseline, and the wrong one for the question this answers:
CAN THE ROBOT GET AROUND THE WORLD.

The differences that matter:

* **Several start poses, chosen across the map.** Rubicon is not one terrain,
  it is a basin, a set of hillsides and a boulder field, and a controller can
  be fine on one and hopeless on another. The default course visits nine.
* **Net displacement, not path length.** A robot shaking itself sideways racks
  up path without going anywhere -- the stock configuration covered 1.73 m of
  path for 0.78 m of progress. Both are reported; the first is the honest one.
* **It re-stands the robot between legs.** A teleport keeps the joint angles it
  had, so resetting a robot that is lying on its back can drop it straight back
  onto its back, and then every remaining leg measures a robot that is already
  over. Legs where it could not be stood up are reported separately rather than
  averaged in.
* **Rollovers and stalls are counted**, because those are the failure modes
  that actually stop a run.

Ground truth comes from the simulator, over a `ros_gz_bridge` this script
starts and stops itself -- 50+ Hz, against the ~4 Hz that repeated `gz model`
calls manage, which is the difference between seeing a rollover and inferring
one.

USAGE
-----
    ros2 run magi_control magi_terrain_trial.py                  # default course
    ros2 run magi_control magi_terrain_trial.py --duration 8 --reps 3
    ros2 run magi_control magi_terrain_trial.py --world flat --course \\
        '[[0,0,0.5,0,1.0,0.0]]'

Each course leg is `[x, y, z, yaw, v, w]`: where to put the robot, and the
twist to hold for `--duration` seconds. `z` should be a few centimetres above
the standing height at that point, not metres -- this measures driving, not
falling.

ALWAYS USE --reps ON TERRAIN. The leg-to-leg spread is large and real: the same
start pose under an identical configuration has produced 2.7 m and 5.4 m,
because a few centimetres of initial drift puts the robot on a different rock.
The reference figures below are pooled over 3-5 reps of the whole course.

REFERENCE, default course, 8 s per leg, `stance_controller:=stabilizer`
-----------------------------------------------------------------------
    configuration                reps  legs  stood  net disp   path  upright
    stock (before the fixes)        3    36     29    0.78 m  1.73 m   25/29
    control fixes only              2    24     21    1.52 m  1.81 m   19/21
    control + rebuilt heightmap     5    60     52    2.18 m  2.59 m   43/52
"""

import argparse
import atexit
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

MODEL = "magi_go2w"
GT_TOPIC = "/magi/ground_truth_pose"

# (x, y, z, yaw, v, w). Nine start poses spread over the map, picked so the
# terrain slope under the robot spans 3-23 deg at its own footprint scale and
# nothing steeper than 27 deg lies on the 6 m ahead of it -- i.e. ground the
# machine ought to be able to cross -- plus two arcs and a spin.
DEFAULT_COURSE = [
    [1.18, 10.90, 3.15, 4.7124, 1.0, 0.0],
    [-5.11, 8.37, 3.82, 1.0472, 1.0, 0.0],
    [-7.68, -1.31, 4.38, 4.1888, 1.0, 0.0],
    [-6.77, 4.22, 4.12, 0.0000, 1.0, 0.0],
    [-3.05, 2.16, 3.71, 0.5236, 1.0, 0.0],
    [5.84, 9.00, 2.68, 5.7596, 1.0, 0.0],
    [3.57, -8.10, 3.31, 5.2360, 1.0, 0.0],
    [-10.33, -8.30, 4.90, 2.0944, 1.0, 0.0],
    [9.30, -9.49, 3.87, 1.0472, 1.0, 0.0],
    [1.18, 10.90, 3.15, 4.7124, 0.8, 0.5],
    [-6.77, 4.22, 4.12, 0.0000, 0.8, -0.5],
    [-5.11, 8.37, 3.82, 1.0472, 0.0, 1.0],
]

OVER = math.radians(60.0)      # past this the run is a rollover, not a lean
UPRIGHT = math.radians(25.0)   # a reset has to land inside this to count


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def start_pose_bridge(world):
    """Bridge the simulator's own pose output, and take it down again on exit.

    ros_gz_bridge drops the entity names when it converts gz.msgs.Pose_V to a
    TFMessage, but Gazebo publishes the model pose first and its links (which
    are model-relative) after it, so transforms[0] is the model in world
    coordinates. That is all this needs.
    """
    proc = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         f"/world/{world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
         "--ros-args", "-r", f"/world/{world}/dynamic_pose/info:={GT_TOPIC}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    atexit.register(lambda: _kill(proc))
    return proc


def _kill(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass


class Trial(Node):
    def __init__(self, world, cmd_topic):
        super().__init__("magi_terrain_trial")
        self.world = world
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(TFMessage, GT_TOPIC, self._on_gt, 50)
        self.msg = Twist()
        self.gt = None                # (t, x, y, z, roll, pitch, yaw)
        self.create_timer(0.02, lambda: self.pub.publish(self.msg))

    def _on_gt(self, msg):
        if not msg.transforms:
            return
        tf = msg.transforms[0].transform
        r, p, y = quat_to_rpy(tf.rotation.x, tf.rotation.y,
                              tf.rotation.z, tf.rotation.w)
        self.gt = (time.time(), tf.translation.x, tf.translation.y,
                   tf.translation.z, r, p, y)

    def reset(self, x, y, z, yaw):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000",
             "--req", f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}},'
                      f' orientation: {{z: {qz}, w: {qw}}}'],
            capture_output=True, timeout=15)

    def fresh_gt(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.gt and time.time() - self.gt[0] < 0.5:
                return True
            time.sleep(0.05)
        return False


def one_leg(node, leg, duration, settle):
    x0, y0, z0, yaw0 = leg[0], leg[1], leg[2], leg[3]
    v, w = leg[4], leg[5]
    node.msg = Twist()

    for attempt in range(4):
        node.reset(x0, y0, z0 + 0.10 * attempt, yaw0)
        time.sleep(settle)
        if not node.fresh_gt():
            return {"error": "no ground truth"}
        if max(abs(node.gt[4]), abs(node.gt[5])) < UPRIGHT:
            break
    else:
        return {"error": "could not be stood up", "start": [x0, y0, yaw0]}

    node.msg.linear.x, node.msg.angular.z = v, w
    t0 = time.time()
    samples = []
    while time.time() - t0 < duration:
        time.sleep(0.05)
        if node.gt:
            samples.append(node.gt)
    node.msg = Twist()
    time.sleep(0.5)

    if len(samples) < 5:
        return {"error": "too few samples"}

    path = 0.0
    for a, b in zip(samples, samples[1:]):
        path += math.hypot(b[1] - a[1], b[2] - a[2])
    # Stalled = moved under 2 cm across a half-second window. Counting it this
    # way rather than "speed below X" keeps a robot that is vibrating in place
    # from being credited with motion.
    win, stalled = 10, 0.0
    for i in range(len(samples) - win):
        a, b = samples[i], samples[i + win]
        if math.hypot(b[1] - a[1], b[2] - a[2]) < 0.02:
            stalled += (b[0] - a[0]) / win

    first, last = samples[0], samples[-1]
    span = last[0] - first[0]
    rolls = [abs(s[4]) for s in samples]
    pitches = [abs(s[5]) for s in samples]
    return {
        "start": [round(c, 2) for c in (x0, y0, yaw0)],
        "cmd": [v, w],
        "dist": round(math.hypot(last[1] - first[1], last[2] - first[2]), 3),
        "path": round(path, 3),
        "speed": round(path / span, 3),
        "max_roll": round(math.degrees(max(rolls)), 1),
        "max_pitch": round(math.degrees(max(pitches)), 1),
        "stall_s": round(stalled, 2),
        "over": bool(max(rolls) > OVER or max(pitches) > OVER),
        "dz": round(last[3] - first[3], 3),
    }


def summarise(results, duration):
    done = [r for r in results if not r.get("error")]
    if not done:
        print("no legs completed")
        return
    n = len(done)
    print("\n--- summary ---")
    print(f"legs attempted   {len(results)}")
    print(f"stood up         {n}")
    print(f"upright at end   {sum(0 if r['over'] else 1 for r in done)}/{n}")
    print(f"net displacement {sum(r['dist'] for r in done) / n:.2f} m "
          f"per {duration:.0f} s leg")
    print(f"path length      {sum(r['path'] for r in done) / n:.2f} m")
    print(f"stalled          {sum(r['stall_s'] for r in done):.1f} s "
          f"of {n * duration:.0f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="rubicon",
                    help="world name, for the set_pose service")
    ap.add_argument("--cmd-topic", default="/cmd_vel",
                    help="/cmd_vel goes through the stabilizer's governor; "
                         "/wheel_controller/cmd_vel_unstamped bypasses it")
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--course", default="",
                    help="JSON list of [x, y, z, yaw, v, w]; default is the "
                         "nine-pose Rubicon course")
    ap.add_argument("--out", default="", help="write the per-leg JSON here")
    args = ap.parse_args()

    course = json.loads(args.course) if args.course else DEFAULT_COURSE

    bridge = start_pose_bridge(args.world)
    rclpy.init()
    node = Trial(args.world, args.cmd_topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(2.0)
    if not node.fresh_gt(timeout=10.0):
        print(f"no ground truth on {GT_TOPIC} -- is the world called "
              f"'{args.world}'?", file=sys.stderr)
        _kill(bridge)
        return 1

    results = []
    for rep in range(args.reps):
        for leg in course:
            r = one_leg(node, leg, args.duration, args.settle)
            r["rep"] = rep
            results.append(r)
            print(json.dumps(r), flush=True)

    summarise(results, args.duration)
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(results, fh, indent=1)

    node.destroy_node()
    rclpy.shutdown()
    _kill(bridge)
    return 0


if __name__ == "__main__":
    sys.exit(main())
