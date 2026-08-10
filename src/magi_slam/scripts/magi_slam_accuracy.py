#!/usr/bin/env python3
"""Compare the SLAM pose against Gazebo ground truth over a driven segment.

Drives a segment and reports how far each estimator thinks it travelled versus
how far it actually did:

    map -> base    the SLAM pose (EKF odometry plus RTAB-Map's correction)
    odom -> base   the EKF alone, for reference
    gz             ground truth

Deltas are compared rather than absolute poses, because the map frame is
anchored wherever RTAB-Map happened to initialise and has no reason to coincide
with the world origin.

    ros2 run magi_slam magi_slam_accuracy.py --linear 0.5 --angular 0.3 --duration 10
"""

import argparse
import math
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

CMD_TOPIC = "/wheel_controller/cmd_vel_unstamped"


def ground_truth():
    out = subprocess.run(["gz", "model", "-m", "magi_go2w", "-p"],
                         capture_output=True, text=True, timeout=20).stdout
    out = re.sub(r"\x1b\[[0-9;]*m", "", out)
    lines = out.splitlines()
    i = next(k for k, ln in enumerate(lines) if "- Pose" in ln)
    xyz = [float(v) for v in
           re.match(r"\s*\[([-\d.eE+ ]+)\]", lines[i + 1]).group(1).split()]
    rpy = [float(v) for v in
           re.match(r"\s*\[([-\d.eE+ ]+)\]", lines[i + 2]).group(1).split()]
    return xyz[0], xyz[1], rpy[2]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self, lin, ang):
        super().__init__("magi_slam_accuracy")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.msg = Twist()
        self.msg.linear.x = lin
        self.msg.angular.z = ang
        self.create_timer(0.02, lambda: self.pub.publish(self.msg))

    def pose(self, parent, child="base"):
        try:
            t = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))


def delta(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--linear", type=float, default=0.5)
    ap.add_argument("--angular", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=10.0)
    a = ap.parse_args()

    rclpy.init()
    n = Driver(a.linear, a.angular)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    deadline = time.time() + 15
    while time.time() < deadline:
        if n.pose("map") and n.pose("odom"):
            break
        time.sleep(0.5)

    m0, o0, g0 = n.pose("map"), n.pose("odom"), ground_truth()
    if m0 is None:
        raise SystemExit("no map -> base transform; is SLAM running?")
    time.sleep(a.duration)
    m1, o1, g1 = n.pose("map"), n.pose("odom"), ground_truth()

    n.msg = Twist()
    time.sleep(1.0)
    n.destroy_node()
    rclpy.shutdown()

    dg, dm, do = delta(g0, g1), delta(m0, m1), delta(o0, o1)

    def wrap(x):
        return math.atan2(math.sin(x), math.cos(x))

    print(f"driven {a.duration:.0f} s at linear {a.linear}, angular {a.angular}\n")
    print(f"{'source':<16}{'distance':>10}{'err':>9}{'yaw':>9}{'yaw err':>9}")
    print(f"{'ground truth':<16}{dg:>10.3f}{'--':>9}"
          f"{math.degrees(wrap(g1[2]-g0[2])):>9.1f}{'--':>9}")
    for name, d, p0, p1 in (("SLAM (map)", dm, m0, m1), ("EKF (odom)", do, o0, o1)):
        dy = math.degrees(wrap(p1[2] - p0[2]) - wrap(g1[2] - g0[2]))
        print(f"{name:<16}{d:>10.3f}{d-dg:>+9.3f}"
              f"{math.degrees(wrap(p1[2]-p0[2])):>9.1f}{dy:>+9.1f}")


if __name__ == "__main__":
    main()
