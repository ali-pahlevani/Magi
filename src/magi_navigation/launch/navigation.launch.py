"""Nav2 for the MAGI Go2W.

THIS FILE IS THE NAVIGATION COMPONENT ONLY. Like magi_slam/slam.launch.py it
starts no simulation, no robot and no controllers -- it expects `map -> odom`
and `odom -> base` to already be on the wire. The runnable entry point is

    ros2 launch magi_launch magi_nav.launch.py

which brings up the world, the robot, the controllers, the EKF, RTAB-Map in
localization mode against a saved map, and then this.

WHAT IT STARTS

    map_server           serves <map_dir>/<map_name>.yaml, saved by a mapping
                         run, as /map with transient-local durability
    obstacles_detection  ground/obstacle split of the live cloud, feeding the
                         costmaps' obstacle layer. OFF by default -- see the
                         header of config/nav2.yaml for the measurements
    planner_server       NavFn on the global costmap
    controller_server    Regulated Pure Pursuit + both costmaps
    behavior_server      spin / back up / drive on heading / wait
    bt_navigator         the NavigateToPose action, and the /goal_pose
                         subscription behind RViz's "2D Goal Pose" button
    lifecycle_manager    brings all of the above up, in that order

NOT amcl: RTAB-Map is the localiser and already owns `map -> odom`. See the
header of config/nav2.yaml.

WHERE THE TWIST GOES

Nav2's controller publishes on /cmd_vel, which is magi_stabilizer's governed
input -- the same topic teleop uses, and the only path with tipover protection
on it. That does mean Nav2 and keyboard teleop are two writers on one topic if
both are running, so magi_nav.launch.py leaves teleop off by default. There is
no velocity smoother in the chain either: the stabilizer's governor and the
diff-drive controller's acceleration limits are already two layers of the same
thing, and a third would only make the robot slower to respond to the first
two.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Order matters: the lifecycle manager activates these in sequence, and the
# costmaps inside controller_server and planner_server block on /map until the
# static layer has one. map_server first is not cosmetic.
MANAGED = [
    "map_server",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
]


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("magi_navigation")

    params = LaunchConfiguration("params_file").perform(context)
    if not params:
        params = os.path.join(share, "config", "nav2.yaml")

    # A list, because the live obstacle layer is an overlay on top of the base
    # configuration rather than a variant of it. Later files win, and only the
    # two `plugins` lists differ.
    live_obstacles = LaunchConfiguration("local_obstacles").perform(
        context).lower() in ("true", "1")
    params = [params]
    if live_obstacles:
        params.append(os.path.join(share, "config", "local_obstacles.yaml"))

    map_yaml = LaunchConfiguration("map_yaml").perform(context).strip()
    if not map_yaml:
        map_dir = os.path.expanduser(
            LaunchConfiguration("map_dir").perform(context))
        map_yaml = os.path.join(
            map_dir, LaunchConfiguration("map_name").perform(context) + ".yaml")
    map_yaml = os.path.expanduser(map_yaml)

    if not os.path.isfile(map_yaml):
        # Worth failing loudly on. Without it map_server comes up, fails to
        # activate, and the whole lifecycle chain stalls with the real reason
        # buried several screens up in one node's log.
        raise RuntimeError(
            "no saved map at %s.\n"
            "Map the world first:\n"
            "    ros2 launch magi_launch magi_test.launch.py slam:=true\n"
            "drive it around, then Ctrl-C -- that writes the map." % map_yaml)

    common = {"use_sim_time": True}

    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=params + [dict(common, yaml_filename=map_yaml)],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=params,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=params,
            # controller_server publishes the final twist. Straight to
            # /cmd_vel, which is the governed input -- see the module header.
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=params,
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=params,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": MANAGED,
                # The costmaps' first static-map wait can be slow while the
                # simulation is still building, and a bounce would restart the
                # whole chain rather than wait it out.
                "bond_timeout": 10.0,
            }],
        ),
    ]

    # Ground/obstacle segmentation for the costmaps' live layer. Same
    # Grid/ settings as the mapping run in magi_slam/config/rtabmap.yaml, so
    # the live layer and the static map agree on what counts as a slope --
    # a mismatch here shows up as the robot refusing to drive up terrain its
    # own map says is drivable.
    if live_obstacles:
        nodes.append(
            Node(
                package="rtabmap_util",
                executable="obstacles_detection",
                name="obstacles_detection",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    # The segmentation plane is z = 0 of THIS frame, which is
                    # the whole reason base_footprint exists.
                    "frame_id": "base_footprint",
                    "wait_for_transform": 0.5,
                    "Grid/Sensor": "0",
                    "Grid/CellSize": "0.1",
                    "Grid/NormalsSegmentation": "true",
                    "Grid/MaxGroundAngle": "45",
                    "Grid/NormalK": "20",
                    "Grid/ClusterRadius": "0.5",
                    "Grid/MinClusterSize": "10",
                    "Grid/MaxObstacleHeight": "1.5",
                    "Grid/RangeMin": "0.6",
                    # 12 m rather than the map's 20: the local costmap is 6 m
                    # square, so anything past this is segmented and then
                    # thrown away.
                    "Grid/RangeMax": "12.0",
                    # NO noise filtering. A radius smaller than the voxel the
                    # cloud was just filtered to has no neighbours to count, so
                    # every point fails the test and the node emits an empty
                    # cloud. Measured: 0.05 m radius with 2 neighbours took the
                    # output from 1379 obstacle points per scan to 5, silently
                    # -- no warning, no error, just a costmap that stopped
                    # updating. If this is ever wanted, the radius has to be
                    # well above Grid/CellSize.
                }],
                remappings=[("cloud", "/lidar/points")],
            )
        )

    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1"):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="magi_nav_rviz",
                output="screen",
                parameters=[common],
                arguments=["-d", os.path.join(share, "rviz", "magi_nav.rviz")],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_dir", default_value="~/magi_maps",
                description="Directory holding the saved maps.",
            ),
            DeclareLaunchArgument(
                "map_name", default_value="rubicon",
                description="Map to navigate on, without an extension.",
            ),
            DeclareLaunchArgument(
                "map_yaml", default_value="",
                description=(
                    "Override the map file outright. Empty derives it from "
                    "map_dir and map_name, which is the normal case."
                ),
            ),
            DeclareLaunchArgument(
                "params_file", default_value="",
                description="Nav2 parameters. Empty uses this package's.",
            ),
            DeclareLaunchArgument(
                "local_obstacles", default_value="false",
                description=(
                    "Add the live lidar obstacle layer to the costmaps. OFF by "
                    "default: this lidar returns 1.6% of its scan from ground "
                    "level, which is not enough to raytrace free space open, "
                    "and the layer walls the robot in. See config/nav2.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false",
                description=(
                    "Open the navigation RViz layout. Off by default because "
                    "magi_nav.launch.py runs one already."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
