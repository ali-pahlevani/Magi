"""Keyboard teleoperation for the MAGI Go2W.

teleop_twist_keyboard needs a real terminal on stdin, so it is launched inside
its own terminal window. If that is awkward in your setup, run it by hand and
skip this launch file entirely:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
        -r /cmd_vel:=/cmd_vel

WHERE TO PUBLISH

    /cmd_vel                              through magi_stabilizer's governor,
                                          which is the default and the only
                                          path with tipover protection on it
    /wheel_controller/cmd_vel_unstamped   straight to the wheels, no governing

wheel_controller runs with use_stamped_vel:=false, so it listens for a plain
geometry_msgs/Twist on ~/cmd_vel_unstamped either way. Publishing there
directly while magi_stabilizer is running means two writers on one topic, so
use the default unless the stabilizer is off.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "terminal",
                default_value="gnome-terminal --",
                description="Terminal emulator prefix used to give teleop a stdin.",
            ),
            DeclareLaunchArgument(
                "cmd_topic",
                default_value="/cmd_vel",
                description=(
                    "Where to publish the twist. The default goes through the "
                    "stabilizer's governor; set it to "
                    "/wheel_controller/cmd_vel_unstamped to drive the wheels "
                    "directly when the stabilizer is not running."
                ),
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="magi_teleop",
                output="screen",
                prefix=LaunchConfiguration("terminal"),
                parameters=[{"use_sim_time": True}],
                remappings=[("/cmd_vel", LaunchConfiguration("cmd_topic"))],
            ),
        ]
    )
