"""Start Gazebo Sim with the offline Rubicon world, plus the clock bridge.

This launches the simulator only; use magi_bringup to also spawn the robot and
bring the controllers up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gz_resource_path():
    """Every directory Gazebo must search for models and package:// meshes.

    Gazebo resolves `package://<pkg>/<rest>` by looking for `<pkg>/<rest>`
    beneath each resource path, so the *share* directory of each install prefix
    has to be listed -- that is what makes the robot meshes load.
    """
    paths = [os.path.join(get_package_share_directory("magi_gazebo"), "models")]

    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share")
        if prefix and os.path.isdir(share):
            paths.append(share)

    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    seen, ordered = set(), []
    for path in paths:
        if path and path not in seen:
            seen.add(path)
            ordered.append(path)
    return os.pathsep.join(ordered)


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")
    verbosity = LaunchConfiguration("verbosity").perform(context)
    paused = LaunchConfiguration("paused").perform(context).lower() in ("true", "1")

    if not os.path.isabs(world):
        world = os.path.join(
            get_package_share_directory("magi_gazebo"), "worlds", world
        )

    # -s runs the server without a GUI; -r starts the world running immediately.
    gz_args = [] if gui else ["-s", "--headless-rendering"]
    if not paused:
        gz_args.append("-r")

    engine = LaunchConfiguration("physics_engine").perform(context)
    if engine:
        gz_args += ["--physics-engine", engine]

    gz_args += ["-v", verbosity, world]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": " ".join(gz_args)}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="magi_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory("magi_gazebo"),
                    "config",
                    "gz_bridge.yaml",
                ),
                "use_sim_time": True,
            }
        ],
    )

    return [gz_sim, clock_bridge]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="rubicon.sdf",
                description="World file: a name inside magi_gazebo/worlds, or an absolute path.",
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Run the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start the world paused instead of running.",
            ),
            DeclareLaunchArgument(
                "verbosity", default_value="3", description="Gazebo verbosity, 0-4."
            ),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="",
                description=(
                    "Physics engine plugin; empty uses the Gazebo default (dartsim). "
                    "Try gz-physics-bullet-featherstone-plugin for different "
                    "wheel-on-heightmap contact behaviour."
                ),
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path()),
            OpaqueFunction(function=launch_setup),
        ]
    )
