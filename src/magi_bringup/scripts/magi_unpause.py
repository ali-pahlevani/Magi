#!/usr/bin/env python3
"""Release the paused simulation once the controllers are ready for it.

The legs are torque controlled, so they carry no load until leg_controller is
active, and any physics stepped before that drops the robot on its belly. The
simulation is therefore started paused and released only when the controllers
are in place.

This used to be a fixed timer, which is a race: how long Gazebo needs to load
the world, the 342 MB terrain, the camera and the lidar varies with machine
load and disk cache. When the timer won, the robot stood up; when it lost, the
unpause landed before the spawners had configured anything, physics ran with
limp legs, and the robot ended up collapsed with its wheels spinning
uselessly against the ground.

So instead of guessing, wait for the actual condition: every expected
controller loaded and configured. Then pause briefly so the spawners can issue
their activation requests, and unpause. Those switches complete on the first
physics tick, and no physics ever runs with limp legs.

Gazebo answers `list_controllers` while paused because gz_ros2_control spins
the node on its own thread; only the controller_manager *update loop* is
stalled, which is exactly why the switches need a tick to land.
"""

import subprocess
import sys

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node

READY_STATES = ("inactive", "active")


class Unpauser(Node):
    def __init__(self):
        super().__init__("magi_unpause")

        self.declare_parameter("world", "rubicon")
        self.declare_parameter(
            "controllers",
            ["joint_state_broadcaster", "imu_sensor_broadcaster",
             "leg_controller", "wheel_controller"])
        # Grace period after the controllers are configured, so the spawners
        # get their activation requests in before physics starts.
        self.declare_parameter("settle_time", 1.5)
        self.declare_parameter("timeout", 120.0)
        # Poll gently. controller_manager serves this on a single-threaded
        # executor and the four spawners are hammering the same service; an
        # aggressive poll here starves them and they die with
        # "Could not successfully call service .../list_controllers".
        self.declare_parameter("poll_period", 2.0)

        self.world = self.get_parameter("world").value
        self.expected = list(self.get_parameter("controllers").value)
        self.settle = float(self.get_parameter("settle_time").value)
        self.timeout = float(self.get_parameter("timeout").value)

        self.client = self.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.start = self.get_clock().now()
        self.get_logger().info(
            f"waiting for {len(self.expected)} controllers before unpausing "
            f"world '{self.world}'")

    def _elapsed(self):
        return (self.get_clock().now() - self.start).nanoseconds * 1e-9

    def ready(self):
        """True once every expected controller is loaded and configured."""
        if not self.client.service_is_ready():
            return False
        future = self.client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            # Drop it rather than leaving the request outstanding; otherwise
            # each poll piles another pending call onto the same server.
            self.client.remove_pending_request(future)
            return False
        states = {c.name: c.state for c in future.result().controller}
        missing = [c for c in self.expected
                   if states.get(c) not in READY_STATES]
        if missing:
            self.get_logger().info(f"still waiting on: {', '.join(missing)}",
                                   throttle_duration_sec=5.0)
            return False
        return True

    def unpause(self):
        result = subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/control",
             "--reqtype", "gz.msgs.WorldControl",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000",
             "--req", "pause: false"],
            capture_output=True, text=True, timeout=20)
        ok = "true" in result.stdout.lower()
        self.get_logger().info(
            f"unpaused world '{self.world}': {result.stdout.strip() or result.stderr.strip()}"
            if ok else
            f"UNPAUSE FAILED for world '{self.world}': "
            f"{result.stdout.strip()} {result.stderr.strip()}")
        return ok


def main():
    rclpy.init()
    node = Unpauser()

    # Use wall time: sim time is frozen while paused, so a sim-clock wait here
    # would never advance.
    import time
    deadline = time.time() + node.timeout
    while rclpy.ok() and time.time() < deadline:
        if node.ready():
            break
        rclpy.spin_once(node, timeout_sec=node.get_parameter("poll_period").value)
    else:
        node.get_logger().error(
            "timed out waiting for controllers; unpausing anyway so the "
            "simulation is not left frozen. Expect the robot to be collapsed.")

    time.sleep(node.settle)
    node.unpause()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
