"""Everything built so far, in one command, set up to inspect odometry.

    ros2 launch magi_launch magi_test.launch.py

Brings up:
    * Gazebo with the offline Rubicon world
    * the Go2W, spawned and stood up
    * ros2_control  (leg torque control, skid-steer wheels, IMU broadcaster)
    * state estimation (IMU conditioner, leg odometry, robot_localization EKF)
    * the closed-loop balance controller (balance:=false for the fixed stance)
    * keyboard teleop, so the robot can be driven by hand
    * RViz, laid out to match what is running

For 3D SLAM:

    ros2 launch magi_launch magi_test.launch.py slam:=true

which also switches RViz to the SLAM layout -- Fixed Frame `map`, the 3D point
cloud on /cloud_map and the 2D occupancy grid on /map, both from the same
RTAB-Map graph. Pass rviz_config:=<path> to override that choice.

The simulation stack is delegated to magi_bringup/magi_sim.launch.py, which
owns the paused-start sequencing the torque-controlled legs depend on. This
file only chooses the configuration: teleop on, and RViz pointed at the
odometry layout instead of the default one.

NOTE it supplies `rviz_config` rather than running an RViz of its own.
IncludeLaunchDescription does not push a configuration scope, so a second RViz
here would mean two `rviz` arguments sharing one context and the child's
declaration would clobber the parent's -- which is exactly the bug that made
RViz silently fail to appear. Wrapping the include in a scoped GroupAction
fixes the leak but breaks the child's deferred TimerActions, whose
LaunchConfigurations are gone by the time they fire. One RViz, configured from
here, avoids both.

RViz is laid out to make odometry *checkable* rather than pretty:

    Fixed Frame is `odom`, and the camera does NOT follow the robot. If the view
    tracked `base` the robot would sit still while the world slid past, which
    hides exactly what we want to see.

    Two odometry trails are drawn at once. Green is /odometry/filtered from the
    EKF; red is the raw /wheel_controller/odom. They start together and the red
    one visibly swings away as soon as you turn, because wheel odometry cannot
    see skid-steer scrub and over-reads yaw by 67-90%.

Both layouts also draw the balance controller's support polygon, centre of
pressure and centre of mass, so the stance can be watched while driving.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_rviz_config(context, *args, **kwargs):
    """Pick the RViz layout to match what is actually being run.

    The two layouts answer different questions and are anchored on different
    frames: the odometry one on `odom`, with the EKF and raw-wheel trails; the
    SLAM one on `map`, with the 3D cloud and the 2D grid. Running SLAM under
    the odometry layout shows no map at all, which used to be a manual pairing
    the caller had to remember. An explicit rviz_config always wins.
    """
    explicit = LaunchConfiguration("rviz_config").perform(context)
    if explicit:
        return [SetLaunchConfiguration("rviz_config", explicit)]

    slam = LaunchConfiguration("slam").perform(context).lower() in ("true", "1")
    package = "magi_slam" if slam else "magi_launch"
    layout = "magi_slam.rviz" if slam else "magi_odometry.rviz"
    return [
        SetLaunchConfiguration(
            "rviz_config",
            os.path.join(get_package_share_directory(package), "rviz", layout),
        )
    ]


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("magi_bringup"),
                "launch",
                "magi_sim.launch.py",
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "teleop": LaunchConfiguration("teleop"),
            "balance": LaunchConfiguration("balance"),
            "localization": "true",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
        }.items(),
    )

    # Publishes map -> odom on top of the EKF's odom -> base. Started after the
    # simulation so its first scan arrives with the clock already running.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("magi_slam"), "launch", "slam.launch.py"
            )
        ),
        launch_arguments={
            "localization": LaunchConfiguration("slam_localization"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="rubicon.sdf"),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Show the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true",
                description="Open RViz.",
            ),
            DeclareLaunchArgument(
                "slam", default_value="false",
                description="Run RTAB-Map 3D lidar SLAM (publishes map -> odom).",
            ),
            DeclareLaunchArgument(
                "slam_localization", default_value="false",
                description="Localise against the stored map instead of mapping.",
            ),
            # Empty means "choose to match `slam`" -- see _resolve_rviz_config.
            # Set it to a path to override that.
            DeclareLaunchArgument(
                "rviz_config", default_value="",
                description=(
                    "RViz layout. Empty selects the odometry layout, or the "
                    "SLAM layout (3D cloud + 2D grid, on `map`) when slam:=true."
                ),
            ),
            DeclareLaunchArgument(
                "teleop", default_value="true",
                description="Open keyboard teleop so the robot can be driven.",
            ),
            DeclareLaunchArgument(
                "balance", default_value="true",
                description=(
                    "Run the closed-loop balance controller. false falls back "
                    "to the fixed stance, for A/B comparison only."
                ),
            ),
            DeclareLaunchArgument("x", default_value="4.0"),
            DeclareLaunchArgument("y", default_value="-0.5"),
            DeclareLaunchArgument("z", default_value="1.80"),
            OpaqueFunction(function=_resolve_rviz_config),
            simulation,
            # Held back so the EKF is already producing odom -> base, and the
            # robot has finished standing up, before RTAB-Map tries to attach
            # map -> odom to it. 45 s rather than 30: at 30 the first scan
            # could arrive while TF was still settling, and rtabmap does not
            # recover from that -- it stays up but silent for the whole
            # session. See wait_for_transform in magi_slam/config/rtabmap.yaml,
            # which is the other half of the fix.
            TimerAction(period=45.0, actions=[slam]),
        ]
    )
