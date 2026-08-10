"""Keyboard teleoperation for the MAGI Go2W.

teleop_twist_keyboard needs a real terminal on stdin, so it is launched inside
its own terminal window. If that is awkward in your setup, run it by hand and
skip this launch file entirely:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
        -r /cmd_vel:=/wheel_controller/cmd_vel_unstamped

wheel_controller runs with use_stamped_vel:=false, so it listens for a plain
geometry_msgs/Twist on ~/cmd_vel_unstamped.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CMD_VEL_TOPIC = "/wheel_controller/cmd_vel_unstamped"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "terminal",
                default_value="gnome-terminal --",
                description="Terminal emulator prefix used to give teleop a stdin.",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="magi_teleop",
                output="screen",
                prefix=LaunchConfiguration("terminal"),
                parameters=[{"use_sim_time": True}],
                remappings=[("/cmd_vel", CMD_VEL_TOPIC)],
            ),
        ]
    )
