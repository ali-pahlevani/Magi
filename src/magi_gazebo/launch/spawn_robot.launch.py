"""Publish the MAGI Go2W description and spawn it into a running Gazebo.

The default spawn point sits in the flat basin near the middle of the Rubicon
terrain (terrain height there is ~1.33 m); the robot is dropped a few
centimetres above its standing height so it settles onto the ground rather than
being born intersecting it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    if not controllers_file:
        controllers_file = os.path.join(
            get_package_share_directory("magi_control"),
            "config",
            "magi_controllers.yaml",
        )

    xacro_file = os.path.join(
        get_package_share_directory("magi_description"), "urdf", "magi_go2w.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " sim_gazebo:=true",
                " controllers_file:=",
                controllers_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-topic", "robot_description",
            "-name", LaunchConfiguration("robot_name"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    return [robot_state_publisher, spawn]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="magi_go2w"),
            DeclareLaunchArgument("x", default_value="4.0"),
            DeclareLaunchArgument("y", default_value="-0.5"),
            DeclareLaunchArgument(
                "z",
                default_value="1.80",
                description="Spawn height; terrain is ~1.33 m at the default x/y.",
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description="Override the controller_manager YAML baked into the model.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
