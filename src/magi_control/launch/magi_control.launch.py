"""Activate the MAGI Go2W controllers.

The controller_manager lives inside the Gazebo process (created by the
gz_ros2_control system plugin), so this launch file only runs spawners against
it.

All four spawners are started IN PARALLEL rather than chained on each other's
exit. That is required by the paused-start sequence: the legs are torque
controlled, so they are limp until leg_controller is active, and any physics
that runs before then drops the robot. magi_sim.launch.py therefore holds the
simulation paused, lets every spawner get as far as a pending switch request,
and only then unpauses -- all four switches complete on the first physics tick.

Chaining the spawners would break that, because a spawner cannot exit while the
simulation is paused: controller_manager performs the switch in its update
loop, which does not tick.

--switch-timeout is what makes the sequence robust rather than a race. The
default switch timeout is 5 s, and adding the camera and lidar pushed world
initialisation past that: leg_controller loaded first, requested its switch,
and timed out ~1 s before the first physics tick arrived, leaving the robot on
its belly with limp legs while the other three controllers came up fine. A
generous timeout lets every switch simply wait for the unpause.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Gazebo has to load the world, spawn the robot and configure the hardware
# before the controller_manager services exist; be generous.
CM_TIMEOUT = "120"

# How long a spawner will wait for its activation to actually take effect. The
# switch only completes on a physics tick, and the simulation is held paused
# until every spawner is ready, so this must comfortably exceed the pause.
SWITCH_TIMEOUT = "60"

CONTROLLERS = [
    "joint_state_broadcaster",
    "imu_sensor_broadcaster",
    "leg_controller",
    "wheel_controller",
]


def spawner(controller):
    return Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            controller,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CM_TIMEOUT,
            "--switch-timeout",
            SWITCH_TIMEOUT,
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_posture",
                default_value="true",
                description="Stand the robot up on start.",
            ),
            DeclareLaunchArgument(
                "balance",
                default_value="true",
                description=(
                    "Run the closed-loop balance controller. false falls back to "
                    "magi_posture, which sets one fixed stance and never touches "
                    "the legs again -- useful only as an A/B baseline."
                ),
            ),
            *[spawner(name) for name in CONTROLLERS],
            # Exactly one of these may run: they both publish to
            # /leg_controller/joint_trajectory, and two writers would fight.
            Node(
                package="magi_control",
                executable="magi_balance.py",
                name="magi_balance",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    PathJoinSubstitution(
                        [FindPackageShare("magi_control"), "config", "magi_balance.yaml"]
                    ),
                ],
                condition=IfCondition(
                    AndSubstitution(
                        LaunchConfiguration("start_posture"),
                        LaunchConfiguration("balance"),
                    )
                ),
            ),
            Node(
                package="magi_control",
                executable="magi_posture.py",
                name="magi_posture",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(
                    AndSubstitution(
                        LaunchConfiguration("start_posture"),
                        NotSubstitution(LaunchConfiguration("balance")),
                    )
                ),
            ),
        ]
    )
