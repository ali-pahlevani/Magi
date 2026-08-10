"""Activate the MAGI Go2W controllers and the stance controller above them.

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

CHOOSING THE STANCE CONTROLLER

Exactly one node may write /leg_controller/joint_trajectory, so the three
options are mutually exclusive:

    balance:=false                      magi_posture -- one fixed stance, set
                                        once and never touched again. The
                                        baseline an A/B run is measured against.
    stance_controller:=balance          magi_balance -- the original
                                        quasi-static CoP controller.
    stance_controller:=stabilizer       magi_stabilizer -- the dynamic tipover
                        (default)       controller, which ALSO governs the
                                        command path. See its module docstring.

The last one is not a drop-in for the other two at the topic level: it owns the
twist the wheel controller consumes, so teleop and navigation have to publish
to /cmd_vel instead of straight to /wheel_controller/cmd_vel_unstamped. Callers
get that routing from teleop.launch.py's `cmd_topic` argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

# stance_controller value -> (executable, config file or None)
STANCE_NODES = {
    "stabilizer": ("magi_stabilizer.py", "magi_stabilizer.yaml"),
    "balance": ("magi_balance.py", "magi_balance.yaml"),
}


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


def _stance_node(context, *args, **kwargs):
    """Pick the one node that drives the legs.

    Resolved in Python rather than with nested IfConditions: three mutually
    exclusive choices over two arguments is four nested substitutions and one
    easy mistake, and a mistake here means either two nodes fighting over
    /leg_controller/joint_trajectory or none driving it at all.
    """
    if LaunchConfiguration("start_posture").perform(context).lower() not in ("true", "1"):
        return []

    if LaunchConfiguration("balance").perform(context).lower() not in ("true", "1"):
        return [Node(
            package="magi_control",
            executable="magi_posture.py",
            name="magi_posture",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )]

    choice = LaunchConfiguration("stance_controller").perform(context).strip().lower()
    if choice not in STANCE_NODES:
        raise RuntimeError(
            f"stance_controller must be one of {sorted(STANCE_NODES)}, got '{choice}'")
    executable, config = STANCE_NODES[choice]

    return [Node(
        package="magi_control",
        executable=executable,
        name=executable.removesuffix(".py"),
        output="screen",
        parameters=[
            {"use_sim_time": True},
            PathJoinSubstitution(
                [FindPackageShare("magi_control"), "config", config]
            ),
        ],
    )]


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
                    "Run a closed-loop stance controller. false falls back to "
                    "magi_posture, which sets one fixed stance and never "
                    "touches the legs again -- the A/B baseline."
                ),
            ),
            DeclareLaunchArgument(
                "stance_controller",
                default_value="stabilizer",
                choices=sorted(STANCE_NODES),
                description=(
                    "Which closed-loop stance controller to run. 'stabilizer' "
                    "adds command governing and needs teleop routed via "
                    "/cmd_vel; 'balance' is the original, kept for A/B."
                ),
            ),
            *[spawner(name) for name in CONTROLLERS],
            OpaqueFunction(function=_stance_node),
        ]
    )
