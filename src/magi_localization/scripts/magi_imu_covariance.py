#!/usr/bin/env python3
"""Stamp covariances onto the simulated IMU stream.

gz.msgs.IMU carries no covariance fields, so everything ros_gz_bridge produces
on /imu/data_raw has all three covariance matrices set to zero. robot_localization
reads a zero measurement covariance as infinite confidence, which collapses the
state covariance and makes the filter degenerate. A real IMU driver publishes
covariances; this node supplies the ones the simulated sensor cannot.

Values default to the variances of the noise model actually configured in
go2w_sensors.xacro, so the filter is tuned against the noise the sensor really
has:

    angular velocity      sigma 2.0e-4 rad/s      -> var 4.0e-8
    linear acceleration   sigma 1.7e-2 m/s^2      -> var 2.89e-4

Orientation is treated anisotropically on purpose. Roll and pitch are
gravity-referenced and genuinely observable, so they get a tight variance. Yaw
has no absolute reference on this robot -- the Go2's IMU is 6-axis with no
magnetometer, so its yaw is integrated and drifts -- and is given a huge
variance to keep any consumer from trusting it as an absolute heading.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# Yaw is unobservable without a magnetometer; make that explicit rather than
# quietly letting a filter lock onto it.
YAW_VARIANCE = 1.0e6


class ImuCovariance(Node):
    def __init__(self):
        super().__init__("magi_imu_covariance")

        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("angular_velocity_stddev", 2.0e-4)
        self.declare_parameter("linear_acceleration_stddev", 1.7e-2)
        self.declare_parameter("roll_pitch_stddev", 0.0087)   # ~0.5 deg

        gyro = self.get_parameter("angular_velocity_stddev").value ** 2
        accel = self.get_parameter("linear_acceleration_stddev").value ** 2
        rp = self.get_parameter("roll_pitch_stddev").value ** 2

        self._orientation = self._diag(rp, rp, YAW_VARIANCE)
        self._angular = self._diag(gyro, gyro, gyro)
        self._linear = self._diag(accel, accel, accel)

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            Imu, self.get_parameter("output_topic").value, qos)
        self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self._on_imu, qos)

        self.get_logger().info(
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; "
            f"gyro var {gyro:.3g}, accel var {accel:.3g}, "
            f"roll/pitch var {rp:.3g}, yaw var {YAW_VARIANCE:.3g}")

    @staticmethod
    def _diag(a, b, c):
        return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]

    def _on_imu(self, msg):
        msg.orientation_covariance = self._orientation
        msg.angular_velocity_covariance = self._angular
        msg.linear_acceleration_covariance = self._linear
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
