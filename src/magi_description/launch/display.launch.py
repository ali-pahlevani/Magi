"""View the MAGI Go2W description in RViz, with no simulator running.

robot_state_publisher supplies TF; joint_state_publisher_gui gives you sliders
for all 16 actuated joints. Useful for checking meshes, frames and joint limits
in isolation from Gazebo.

    ros2 launch magi_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    # sim_gazebo:=false keeps the ros2_control/Gazebo tags out of the model, so
    # this works without gz_ros2_control being present.
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("magi_description"), "urdf", "magi_go2w.urdf.xacro"]
                ),
                " sim_gazebo:=false",
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Use joint_state_publisher_gui sliders instead of zeros.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("magi_description"), "rviz", "magi_go2w.rviz"]
                    ),
                ],
            ),
        ]
    )
