#!/usr/bin/env python3
"""Compare heading estimates against Gazebo ground truth.

The point of the EKF is that wheel odometry cannot see skid-steer scrub and so
over-reads yaw. This drives the robot and reports, for each source, how much
heading it thinks accumulated versus how much actually did.

    ros2 run magi_localization magi_yaw_compare.py --angular 0.5 --duration 8

Sources compared:
    ground truth   gz model pose, the reference
    wheel odom     /wheel_controller/odom, yaw from wheel speed difference
    EKF            /odometry/filtered, yaw integrated from the IMU gyro
"""

import argparse
import math
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMD_TOPIC = "/wheel_controller/cmd_vel_unstamped"


def gt_yaw():
    out = subprocess.run(["gz", "model", "-m", "magi_go2w", "-p"],
                         capture_output=True, text=True, timeout=20).stdout
    out = re.sub(r"\x1b\[[0-9;]*m", "", out)
    lines = out.splitlines()
    i = next(k for k, ln in enumerate(lines) if "- Pose" in ln)
    rpy = [float(v) for v in
           re.match(r"\s*\[([-\d.eE+ ]+)\]", lines[i + 2]).group(1).split()]
    return rpy[2]


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Compare(Node):
    def __init__(self, lin, ang):
        super().__init__("magi_yaw_compare")
        self.wheel = None
        self.ekf = None
        self.create_subscription(Odometry, "/wheel_controller/odom",
                                 lambda m: setattr(self, "wheel", yaw_of(m)), 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda m: setattr(self, "ekf", yaw_of(m)), 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.msg = Twist()
        self.msg.linear.x = lin
        self.msg.angular.z = ang
        self.create_timer(0.02, lambda: self.pub.publish(self.msg))


def unwrap(total, prev, cur):
    return total + math.atan2(math.sin(cur - prev), math.cos(cur - prev))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--linear", type=float, default=0.0)
    ap.add_argument("--angular", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=8.0)
    a = ap.parse_args()

    rclpy.init()
    n = Compare(a.linear, a.angular)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    while n.wheel is None or n.ekf is None:
        time.sleep(0.2)
    time.sleep(2.0)     # let the command ramp in

    g_prev, w_prev, e_prev = gt_yaw(), n.wheel, n.ekf
    g_tot = w_tot = e_tot = 0.0
    t0 = time.time()
    while time.time() - t0 < a.duration:
        time.sleep(0.25)
        g, w, e = gt_yaw(), n.wheel, n.ekf
        g_tot = unwrap(g_tot, g_prev, g)
        w_tot = unwrap(w_tot, w_prev, w)
        e_tot = unwrap(e_tot, e_prev, e)
        g_prev, w_prev, e_prev = g, w, e
    dt = time.time() - t0

    n.msg = Twist()
    time.sleep(1.0)
    n.destroy_node()
    rclpy.shutdown()

    print(f"commanded {a.angular:.2f} rad/s for {dt:.1f} s "
          f"(linear {a.linear:.2f} m/s)\n")
    print(f"{'source':<14}{'yaw (rad)':>11}{'deg':>9}{'err vs truth':>14}")
    print(f"{'ground truth':<14}{g_tot:>11.3f}{math.degrees(g_tot):>9.1f}"
          f"{'--':>14}")
    for name, val in (("wheel odom", w_tot), ("EKF", e_tot)):
        err = val - g_tot
        pct = 100.0 * err / abs(g_tot) if abs(g_tot) > 1e-6 else float("nan")
        print(f"{name:<14}{val:>11.3f}{math.degrees(val):>9.1f}"
              f"{err:>+9.3f} ({pct:+.0f}%)")


if __name__ == "__main__":
    main()
