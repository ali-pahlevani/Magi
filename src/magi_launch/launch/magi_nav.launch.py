"""Autonomous navigation on a saved map, in one command.

    ros2 launch magi_launch magi_nav.launch.py

Brings up the same simulation stack as magi_test.launch.py -- Gazebo with
Rubicon, the Go2W, ros2_control, the EKF, the stance controller -- and then:

    * RTAB-Map in LOCALIZATION mode, against a map saved by a mapping run.
      It publishes map -> odom and adds nothing to the graph.
    * Nav2: map_server on the saved map, NavFn, Regulated Pure Pursuit,
      the recovery behaviours and the behaviour tree.
    * RViz on the navigation layout.

Then click "2D Goal Pose" in the RViz toolbar and put an arrow on the map. The
robot plans a route and drives there.

MAP FIRST. This launch needs a map that already exists, and refuses to start
without one. To make it:

    ros2 launch magi_launch magi_test.launch.py slam:=true

drive around the area you want to navigate, and Ctrl-C. That writes
~/magi_maps/rubicon.{db,yaml,pgm,ply} -- the database for relocalisation, the
YAML and PGM pair for Nav2, the cloud for looking at.

    map_name:=<name>   navigate a different saved map
    teleop:=true       keyboard teleop as well. Off by default: Nav2 and teleop
                       both publish /cmd_vel, and whichever message landed last
                       is the one the robot obeys.
    gui:=false         headless Gazebo

WHY START WHERE THE MAP STARTED

The spawn defaults match magi_test.launch.py's, and that is load-bearing.
RTAB-Map anchors `map` on the robot's pose at the first keyframe, and
localization starts its search from the map origin, so respawning at the
original spawn point makes the initial guess correct to within a scan. Spawn
somewhere else and localisation has to find itself by loop closure first,
which it may or may not do before Nav2 starts asking where the robot is.
If that happens, "2D Pose Estimate" in RViz feeds /initialpose to RTAB-Map,
which relocalises from it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    simulation = _include(
        "magi_bringup", "magi_sim.launch.py",
        {
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            # One RViz, configured from here, for the reason spelled out in
            # magi_test.launch.py: IncludeLaunchDescription does not push a
            # configuration scope, so a second RViz would mean two `rviz`
            # arguments sharing one context.
            "rviz_config": os.path.join(
                get_package_share_directory("magi_navigation"),
                "rviz", "magi_nav.rviz"),
            "teleop": LaunchConfiguration("teleop"),
            "balance": LaunchConfiguration("balance"),
            "stance_controller": LaunchConfiguration("stance_controller"),
            "localization": "true",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
        },
    )

    localiser = _include(
        "magi_slam", "slam.launch.py",
        {
            "localization": "true",
            "map_dir": LaunchConfiguration("map_dir"),
            "map_name": LaunchConfiguration("map_name"),
            # nav2_map_server owns /map here: it serves the saved file, which
            # exists from t=0 and does not grow under the planner. RTAB-Map's
            # own grid moves aside rather than becoming a second publisher of
            # the same topic -- a fault that shows up only as a map flickering
            # between two versions of itself.
            "grid_topic": "rtabmap/map",
        },
    )

    navigation = _include(
        "magi_navigation", "navigation.launch.py",
        {
            "map_dir": LaunchConfiguration("map_dir"),
            "map_name": LaunchConfiguration("map_name"),
            "local_obstacles": LaunchConfiguration("local_obstacles"),
            "rviz": "false",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="rubicon.sdf"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "teleop", default_value="false",
                description=(
                    "Keyboard teleop alongside Nav2. Off by default: both "
                    "publish /cmd_vel and the robot obeys whichever arrived "
                    "last."
                ),
            ),
            DeclareLaunchArgument("map_dir", default_value="~/magi_maps"),
            DeclareLaunchArgument(
                "map_name", default_value="rubicon",
                description="Saved map to localise and navigate on.",
            ),
            DeclareLaunchArgument(
                "local_obstacles", default_value="false",
                description=(
                    "Add the live lidar obstacle layer to the costmaps. Off by "
                    "default because this lidar barely sees the ground; the "
                    "measurements are in magi_navigation/config/nav2.yaml."
                ),
            ),
            DeclareLaunchArgument("balance", default_value="true"),
            DeclareLaunchArgument(
                "stance_controller", default_value="stabilizer",
                choices=["stabilizer", "balance"],
            ),
            # Matching magi_test.launch.py, and matching where the map was
            # anchored. See the module docstring.
            DeclareLaunchArgument("x", default_value="4.0"),
            DeclareLaunchArgument("y", default_value="-0.5"),
            DeclareLaunchArgument("z", default_value="1.80"),

            simulation,
            # Same 45 s as magi_test.launch.py, and for the same reason: the
            # EKF has to be producing odom -> base and the robot has to have
            # finished standing before RTAB-Map attaches map -> odom to it. A
            # scan that arrives while TF is still settling leaves rtabmap up
            # but permanently silent.
            TimerAction(period=45.0, actions=[localiser]),
            # Nav2 after that again. Its costmaps latch onto the TF tree as
            # they activate, and starting them before map -> odom exists means
            # riding out a stall in the lifecycle chain for no gain.
            TimerAction(period=60.0, actions=[navigation]),
        ]
    )
