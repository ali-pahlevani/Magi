"""3D lidar SLAM for the MAGI Go2W, using RTAB-Map.

THIS FILE IS THE SLAM COMPONENT ONLY. It starts rtabmap and nothing else -- no
Gazebo, no robot, no controllers, no balance controller. It expects the rest of
the stack to be up already, and is normally not launched by hand.

To map, use:

    ros2 launch magi_launch magi_test.launch.py slam:=true

which brings up the simulation, the robot, ros2_control, the EKF, the balance
controller and RViz (switched to the SLAM layout automatically), then includes
this file on a timer once odom -> base is live. Launching this file by itself
against a running stack is still useful for restarting SLAM without restarting
the simulation.

Publishes `map -> odom`. The Phase 2 EKF keeps publishing `odom -> base`, so
between them the full `map -> odom -> base` chain exists and exactly one node
owns each link.

    localization:=true    reuse an existing map instead of building one
    delete_db:=true       start a fresh map (default when mapping)
    map_name:=<name>      which map to build or reuse
    save_map:=false       do not write the Nav2 map files on shutdown

WHERE THE MAP GOES, AND WHEN IT IS SAVED

RTAB-Map streams its graph into a database as it maps, so the map is on disk
continuously and there is no "save" step for it -- quitting is the save. This
file simply points that database at its final home from the start,

    <map_dir>/<map_name>.db          default ~/magi_maps/rubicon.db

rather than at a scratch path that would have to be copied afterwards. Copying
is the thing to avoid: rtabmap flushes on shutdown, so anything that copies the
file while it is exiting can capture a truncated map.

Nav2 cannot read a .db, so magi_map_saver runs alongside and writes the 2D grid
out as the image-plus-YAML pair nav2_map_server loads, plus a .ply of the 3D
cloud, when the launch is stopped. Ctrl-C is the whole procedure. To snapshot
without stopping:

    ros2 service call /magi_map_saver/save std_srvs/srv/Trigger

A mapping run WIPES the database it is pointed at, on purpose: mapping twice
into one graph without closing the loop produces a doubled map. Use
delete_db:=false to genuinely extend an existing map, or a different map_name
to keep both.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def launch_setup(context, *args, **kwargs):
    localization = _flag(context, "localization")
    delete_db = _flag(context, "delete_db")

    map_dir = os.path.expanduser(
        LaunchConfiguration("map_dir").perform(context))
    map_name = LaunchConfiguration("map_name").perform(context)

    database = LaunchConfiguration("database_path").perform(context).strip()
    database = (os.path.expanduser(database) if database
                else os.path.join(map_dir, map_name + ".db"))
    os.makedirs(os.path.dirname(database), exist_ok=True)

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
        # Where localization starts guessing from. RTAB-Map's default is the
        # pose it was at when the database was last written -- wherever the
        # mapping run happened to stop, which is nowhere near the spawn point.
        # ICP would then be asked to close a gap the size of the world, and
        # would not. The robot always respawns at the pose the map was anchored
        # on, so the origin is the correct prior.
        "RGBD/StartAtOrigin": "true" if localization else "false",
    }

    args = []
    if delete_db and not localization:
        # Only ever wipe when mapping. Doing it in localization mode would
        # delete the very map being localised against.
        args.append("--delete_db_on_start")

    # Where rtabmap's own occupancy grid goes. It is /map while mapping, which
    # is what RViz and the map saver read. Under navigation nav2_map_server
    # owns /map -- it serves the saved file, which exists from t=0 and does not
    # grow under the planner's feet -- so the caller moves rtabmap's grid
    # aside. Two publishers on one topic is a silent fault: subscribers take
    # whichever message landed last, and the map appears to flicker between two
    # versions of itself.
    grid_topic = LaunchConfiguration("grid_topic").perform(context)
    remappings = [
        ("scan_cloud", "/lidar/points"),
        ("odom", "/odometry/filtered"),
    ]
    if grid_topic and grid_topic != "map":
        remappings += [("map", grid_topic), ("grid_map", grid_topic + "_grid")]

    nodes = [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[config, overrides],
            remappings=remappings,
            arguments=args,
        ),
    ]

    # Only while mapping. In localization mode the grid on the wire is a replay
    # of the stored map, and saving it back over itself gains nothing while
    # risking a partial map overwriting a complete one.
    if _flag(context, "save_map") and not localization:
        nodes.append(
            Node(
                package="magi_slam",
                executable="magi_map_saver.py",
                name="magi_map_saver",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "map_dir": map_dir,
                    "map_name": map_name,
                    "grid_topic": grid_topic if grid_topic else "map",
                    "cloud_topic": "cloud_map",
                }],
            )
        )

    return nodes


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
                "map_dir", default_value="~/magi_maps",
                description="Directory holding the saved maps.",
            ),
            DeclareLaunchArgument(
                "map_name", default_value="rubicon",
                description="Map to build or reuse, without an extension.",
            ),
            DeclareLaunchArgument(
                "database_path", default_value="",
                description=(
                    "Override the database location. Empty derives it from "
                    "map_dir and map_name, which is the normal case."
                ),
            ),
            DeclareLaunchArgument(
                "save_map", default_value="true",
                description=(
                    "Run magi_map_saver, which writes the Nav2 map files on "
                    "shutdown. Ignored when localizing."
                ),
            ),
            DeclareLaunchArgument(
                "grid_topic", default_value="map",
                description=(
                    "Where rtabmap publishes its occupancy grid. Move it aside "
                    "when something else owns /map, as nav2_map_server does."
                ),
            ),
            DeclareLaunchArgument("rtabmap_config", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
