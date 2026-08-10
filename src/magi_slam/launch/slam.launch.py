"""3D lidar SLAM for the MAGI Go2W, using RTAB-Map.

THIS FILE IS THE SLAM COMPONENT ONLY. It starts rtabmap and nothing else -- no
Gazebo, no robot, no controllers, no balance controller. It expects the rest of
the stack to be up already, and is normally not launched by hand.

To run SLAM on the robot, use:

    ros2 launch magi_launch magi_test.launch.py slam:=true

which brings up the simulation, the robot, ros2_control, the EKF, the balance
controller and RViz (switched to the SLAM layout automatically), then includes
this file on a timer once odom -> base is live. Launching this file by itself
against a running stack is still useful for restarting SLAM without restarting
the simulation.

Publishes `map -> odom`. The Phase 2 EKF keeps publishing `odom -> base`, so
between them the full `map -> odom -> base` chain exists and exactly one node
owns each link.

    localization:=true    reuse an existing database instead of mapping
    delete_db:=true       start a fresh map (default when mapping)

RTAB-Map keeps its graph in a database file. Mapping runs normally overwrite it
so a session starts clean; set delete_db:=false to extend an existing map.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    localization = LaunchConfiguration("localization").perform(context).lower() in (
        "true", "1")
    delete_db = LaunchConfiguration("delete_db").perform(context).lower() in (
        "true", "1")
    database = os.path.expanduser(
        LaunchConfiguration("database_path").perform(context))

    config = LaunchConfiguration("rtabmap_config").perform(context)
    if not config:
        config = os.path.join(
            get_package_share_directory("magi_slam"), "config", "rtabmap.yaml")

    overrides = {
        "database_path": database,
        # Localization mode freezes the graph: RTAB-Map still corrects
        # map -> odom against the stored map but stops adding to it.
        "Mem/IncrementalMemory": "false" if localization else "true",
        "Mem/InitWMWithAllNodes": "true" if localization else "false",
    }

    args = []
    if delete_db and not localization:
        # Only ever wipe when mapping. Doing it in localization mode would
        # delete the very map being localised against.
        args.append("--delete_db_on_start")

    return [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[config, overrides],
            remappings=[
                ("scan_cloud", "/lidar/points"),
                ("odom", "/odometry/filtered"),
            ],
            arguments=args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "localization", default_value="false",
                description="Localise against the stored map instead of mapping.",
            ),
            DeclareLaunchArgument(
                "delete_db", default_value="true",
                description="Wipe the database on start. Ignored when localizing.",
            ),
            DeclareLaunchArgument(
                "database_path", default_value="~/.ros/magi_rtabmap.db",
            ),
            DeclareLaunchArgument("rtabmap_config", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
