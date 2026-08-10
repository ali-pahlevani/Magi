"""State estimation for the MAGI Go2W.

Runs the IMU covariance conditioner and the robot_localization EKF that owns
the odom -> base transform.

    ros2 launch magi_localization localization.launch.py

diff_drive_controller must not also publish odom -> base; magi_controllers.yaml
sets enable_odom_tf: false for exactly this reason. Two publishers of the same
transform makes TF non-deterministic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_ekf = os.path.join(
        get_package_share_directory("magi_localization"), "config", "ekf.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ekf_config",
                default_value=default_ekf,
                description="robot_localization parameter file.",
            ),
            Node(
                package="magi_localization",
                executable="magi_imu_covariance.py",
                name="magi_imu_covariance",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="magi_localization",
                executable="magi_leg_odometry.py",
                name="magi_leg_odometry",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[LaunchConfiguration("ekf_config"),
                            {"use_sim_time": True}],
            ),
        ]
    )
