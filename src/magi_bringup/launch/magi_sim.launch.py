"""Bring up the whole MAGI Go2W simulation.

    ros2 launch magi_bringup magi_sim.launch.py

Starts Gazebo with the offline Rubicon world, spawns the Go2W, activates the
ros2_control controllers, and finally opens RViz already populated. The robot
stands up on its own once leg_controller is active.

Drive it -- either add teleop to this launch:

    ros2 launch magi_bringup magi_sim.launch.py teleop:=true

or run it separately in its own terminal:

    ros2 launch magi_control teleop.launch.py

Change posture with:

    ros2 topic pub --once /magi/posture std_msgs/String "{data: crouch}"

Startup is staged on timers rather than process events, because Gazebo, the
spawn service and the controller_manager all live in one process and none of
them signal readiness by exiting. The GUI gets a longer stagger simply because
it has 342 MB of terrain to build before it can usefully take a spawn; headless
has no renderer and needs none of that slack. Override any of
spawn_delay/controllers_delay/rviz_delay if your machine differs.

ON THE PAUSED START (paused:=true, off by default)

The legs are torque controlled and carry no load until leg_controller is
active, so the obvious worry is that physics stepped before then drops the
robot. It does drop, briefly -- and then the PD stands it straight back up.
Measured with paused:=false: all four controllers active, robot settled at
0.391 m, which is the design stance.

An earlier version held the simulation paused until the controllers were ready.
That was a mistake, for two reasons. It was based on a misdiagnosis: the
unrecoverable collapse seen during Phase 0 came from a wrongly-signed constant
seed torque driving the joints into their limits over five seconds, not from
the limp window. And it could not be made reliable, because activation itself
needs physics ticks: the switch sits pending, controller_manager gives up after
--switch-timeout, and whether the unpause lands first depends on how long
Gazebo took to build the world. On a GUI run that race was lost by two seconds
and the robot stayed on its belly with its wheels spinning.

paused:=true and magi_unpause are kept for cases where the limp window really
must be eliminated, but the default path is simpler and is the one that is
verified.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Seconds after launch to spawn the robot / start controllers / open RViz,
# keyed on whether the Gazebo GUI is rendering.
STAGGER = {
    True: {"spawn": 14.0, "controllers": 19.0, "rviz": 25.0},
    False: {"spawn": 8.0, "controllers": 12.0, "rviz": 17.0},
}


def _include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=arguments,
        condition=condition,
    )


def _gazebo():
    return _include(
        "magi_gazebo",
        "gazebo.launch.py",
        {
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "verbosity": LaunchConfiguration("verbosity"),
            "physics_engine": LaunchConfiguration("physics_engine"),
            "paused": LaunchConfiguration("paused"),
        }.items(),
    )


def _world_name(context):
    """The <world name="..."> inside the world file, needed for the gz service."""
    world = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world):
        world = os.path.join(
            get_package_share_directory("magi_gazebo"), "worlds", world
        )
    match = re.search(r'<world\s+name=["\']([^"\']+)["\']', open(world).read())
    return match.group(1) if match else os.path.splitext(os.path.basename(world))[0]


def _unpause(context):
    """Release the simulation once the controllers are actually configured.

    The legs run on torque, so they are limp until leg_controller is active and
    any physics stepped before that point drops the robot. Starting paused
    removes that window.

    This is a NODE THAT WAITS ON A CONDITION, not a timer. A fixed delay was a
    race: how long Gazebo needs for the world, the 342 MB terrain, the camera
    and the lidar varies with machine load and disk cache. When the timer lost,
    physics started with limp legs and the robot ended up collapsed on its
    belly, wheels spinning against the ground. magi_unpause polls
    list_controllers until everything is loaded, then unpauses.
    """
    return Node(
        package="magi_bringup",
        executable="magi_unpause.py",
        name="magi_unpause",
        output="screen",
        parameters=[{
            "world": _world_name(context),
            # Wall-clocked: sim time is frozen while paused.
            "use_sim_time": False,
        }],
    )


def _spawn():
    return _include(
        "magi_gazebo",
        "spawn_robot.launch.py",
        {
            "robot_name": LaunchConfiguration("robot_name"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )


def _controllers():
    return _include(
        "magi_control",
        "magi_control.launch.py",
        {
            "start_posture": LaunchConfiguration("start_posture"),
            "balance": LaunchConfiguration("balance"),
        }.items(),
    )


def _teleop():
    return _include(
        "magi_control",
        "teleop.launch.py",
        {}.items(),
        condition=IfCondition(LaunchConfiguration("teleop")),
    )


def _localization():
    return _include(
        "magi_localization",
        "localization.launch.py",
        {}.items(),
        condition=IfCondition(LaunchConfiguration("localization")),
    )


# GTK/GIO/locale variables that a snap-packaged terminal (VS Code's integrated
# terminal, for one) points into its own runtime. Qt then loads the snap's GTK
# module, which drags in the snap's libpthread, and RViz dies at startup with
#   symbol lookup error: .../libpthread.so.0: undefined symbol: __libc_pthread_init
# Blanking them makes RViz fall back to the system libraries.
SNAP_LEAKY_VARS = (
    "GTK_PATH",
    "LOCPATH",
    "GIO_MODULE_DIR",
    "GDK_PIXBUF_MODULE_FILE",
    "GSETTINGS_SCHEMA_DIR",
)


def _snap_safe_env():
    """Blank only the vars that actually point into a snap, so a normal shell
    launches with its environment untouched."""
    return {
        var: ""
        for var in SNAP_LEAKY_VARS
        if "/snap/" in os.environ.get(var, "")
    }


def _rviz():
    # Opened last, so by the time the window appears /robot_description has been
    # latched and odom -> base is already publishing: the model is there
    # immediately instead of the user watching TF errors clear.
    return Node(
        package="rviz2",
        executable="rviz2",
        name="magi_rviz",
        output="screen",
        additional_env=_snap_safe_env(),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )


def launch_setup(context, *args, **kwargs):
    """Resolve the stagger once `gui` is known, honouring explicit overrides."""
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")
    paused = LaunchConfiguration("paused").perform(context).lower() in ("true", "1")
    defaults = STAGGER[gui]

    def delay(name):
        override = LaunchConfiguration(f"{name}_delay").perform(context).strip()
        return float(override) if override else defaults[name]

    # The unpauser starts alongside the spawners and blocks on the controllers
    # being configured, so it needs no delay of its own.
    unpause_at = delay("controllers")

    return [
        TimerAction(period=delay("spawn"), actions=[_spawn()]),
        TimerAction(period=delay("controllers"),
                    actions=[_controllers(), _unpause(context)]
                    if paused else [_controllers()]),
        # Localization and RViz are still staggered, but only so their first
        # cycle sees a running clock rather than a stalled one. Correctness no
        # longer depends on these numbers.
        TimerAction(period=unpause_at + 4.0, actions=[_localization()]),
        TimerAction(period=max(delay("rviz"), unpause_at + 6.0),
                    actions=[_rviz(), _teleop()]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="rubicon.sdf"),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Show the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz once the robot and controllers are up.",
            ),
            # Overridable so a parent launch can supply its own layout rather
            # than running a second RViz of its own. Two RViz nodes would mean
            # two `rviz` arguments in one context, and IncludeLaunchDescription
            # is not scoped, so the child's declaration would clobber the
            # parent's and silently disable one of them.
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("magi_description"), "rviz", "magi_go2w.rviz"]
                ),
                description="RViz layout to open.",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="false",
                description="Also open keyboard teleop in its own terminal.",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="true",
                description="Run the EKF that owns odom -> base.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description=(
                    "Hold the simulation paused until the controllers are up, "
                    "via magi_unpause. Not needed by default: the legs recover "
                    "from the brief limp window on their own. See the note in "
                    "the module docstring before turning this on."
                ),
            ),
            DeclareLaunchArgument("verbosity", default_value="3"),
            DeclareLaunchArgument("physics_engine", default_value=""),
            DeclareLaunchArgument("robot_name", default_value="magi_go2w"),
            DeclareLaunchArgument("x", default_value="4.0"),
            DeclareLaunchArgument("y", default_value="-0.5"),
            DeclareLaunchArgument("z", default_value="1.80"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("start_posture", default_value="true"),
            # Passed down explicitly rather than left to magi_control's own
            # default: IncludeLaunchDescription does not push a configuration
            # scope, so an undeclared child argument leaks into this context
            # and cannot be set from a parent launch file.
            DeclareLaunchArgument(
                "balance", default_value="true",
                description=(
                    "Run the closed-loop balance controller. false falls back "
                    "to the fixed stance, for A/B comparison only."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_delay",
                default_value="",
                description="Override the GUI-aware spawn delay, in seconds.",
            ),
            DeclareLaunchArgument("controllers_delay", default_value=""),
            DeclareLaunchArgument("rviz_delay", default_value=""),

            _gazebo(),
            OpaqueFunction(function=launch_setup),
        ]
    )
