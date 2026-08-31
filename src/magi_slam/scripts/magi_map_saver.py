#!/usr/bin/env python3
"""Write the SLAM map to disk, on shutdown or on demand.

WHY THIS EXISTS

RTAB-Map already persists everything it knows in its database, and that
database is the map: the pose graph, every keyframe's scan, and the occupancy
grid rebuilt from them. Point `database_path` at a file, map, quit, and the map
is on disk with nothing else to do. Localisation mode reads it straight back.

What the database is NOT is a map anything else can read. Nav2's static layer
wants a `nav_msgs/OccupancyGrid`, served by `nav2_map_server` from the
image-plus-YAML pair the ROS ecosystem has used since forever, and there is no
path from a .db to that pair without a running RTAB-Map. So this node watches
the grid RTAB-Map publishes while mapping and writes the pair out at the end,
in exactly the format `nav2_map_server` loads. The 3D cloud gets a .ply
alongside it, for looking at without starting a SLAM stack.

The result is one directory per map, holding the same map three ways:

    <map_dir>/<name>.db     the RTAB-Map graph          -> relocalisation
    <map_dir>/<name>.yaml   the 2D grid, + .pgm beside  -> Nav2's static layer
    <map_dir>/<name>.ply    the 3D cloud                -> viewing

Only the last two are written here. The .db is RTAB-Map's own file, written by
RTAB-Map, and it is deliberately left that way -- copying a database out from
under a process that is still flushing it is how you get a truncated map.
slam.launch.py points RTAB-Map at its final location from the start instead.

WHEN IT WRITES

    on shutdown     Ctrl-C the mapping launch and the map is written. This is
                    the default, and the answer to "how do I save the map".
    on request      ros2 service call /magi_map_saver/save std_srvs/srv/Trigger
                    which is the way to snapshot mid-session, or to rescue a
                    grid from a session that is already running.

Shutdown saving works because the last grid received is held in memory: by the
time SIGINT arrives RTAB-Map may already be gone, and asking it for anything
would be too late. Nothing here talks to RTAB-Map at all.

    ros2 run magi_slam magi_map_saver.py --ros-args -p map_name:=rubicon
"""

import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

# Match whatever RTAB-Map publishes with. BEST_EFFORT + VOLATILE is the
# permissive end of both axes, so it connects to a RELIABLE and/or
# TRANSIENT_LOCAL publisher too. Being late to a latched topic costs nothing
# here: the map is republished on every update and this node runs for the whole
# session, so it sees them all.
PERMISSIVE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def _box_sum(a, k):
    """Sum over a k x k window, per cell, via an integral image.

    numpy only, on purpose: scipy.ndimage would do this in one call and is one
    more thing to have installed on a robot to save a map.
    """
    pad = k // 2
    p = np.pad(a, pad + 1, mode="constant")
    integral = p.cumsum(axis=0).cumsum(axis=1)
    lo, hi = 0, k
    n = a.shape
    return (integral[hi:hi + n[0], hi:hi + n[1]]
            - integral[lo:lo + n[0], hi:hi + n[1]]
            - integral[hi:hi + n[0], lo:lo + n[1]]
            + integral[lo:lo + n[0], lo:lo + n[1]])


class MapSaver(Node):
    def __init__(self):
        super().__init__("magi_map_saver")

        def p(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.map_dir = os.path.expanduser(p("map_dir", "~/magi_maps"))
        self.map_name = p("map_name", "rubicon")
        self.save_on_shutdown = p("save_on_shutdown", True)
        self.save_cloud = p("save_cloud", True)
        # nav2_map_server's own defaults, and the thresholds its loader will
        # apply when reading the pair back. Writing with different ones would
        # quietly change which cells come back free.
        self.occupied_thresh = p("occupied_thresh", 0.65)
        self.free_thresh = p("free_thresh", 0.25)

        # The slope layer. See _slope_mask for what it is and why it is here.
        self.slope_layer = p("slope_layer", True)
        self.slope_limit = p("slope_limit", 35.0)
        self.slope_window = p("slope_window", 1.5)
        self.slope_min_coverage = p("slope_min_coverage", 0.15)

        self.grid = None
        self.grid_stamp = None
        self.cloud = None

        self.create_subscription(
            OccupancyGrid, p("grid_topic", "/map"), self._on_grid, PERMISSIVE)
        if self.save_cloud:
            self.create_subscription(
                PointCloud2, p("cloud_topic", "/cloud_map"),
                self._on_cloud, PERMISSIVE)

        self.create_service(Trigger, "~/save", self._on_request)

        self.get_logger().info(
            "map saver ready: will write %s on shutdown"
            % os.path.join(self.map_dir, self.map_name + ".{pgm,yaml,ply}"))

    # ---- inputs ---------------------------------------------------------

    def _on_grid(self, msg):
        first = self.grid is None
        self.grid = msg
        self.grid_stamp = time.time()
        if first:
            self.get_logger().info(
                "first map: %d x %d cells at %.3f m"
                % (msg.info.width, msg.info.height, msg.info.resolution))

    def _on_cloud(self, msg):
        self.cloud = msg

    def _on_request(self, request, response):
        ok, message = self.save("service request")
        response.success = ok
        response.message = message
        return response

    # ---- output ---------------------------------------------------------

    def save(self, reason):
        """Write every product we have. Returns (ok, human-readable summary)."""
        if self.grid is None:
            message = ("no map to save: nothing has been received on the grid "
                       "topic. Is rtabmap running, and mapping?")
            self.get_logger().error(message)
            return False, message

        try:
            os.makedirs(self.map_dir, exist_ok=True)
            stem = os.path.join(self.map_dir, self.map_name)
            # Grid first, but it reads self.cloud for the slope layer, so
            # both products come from the same instant of the same map.
            written = [self._write_grid(stem)]
            if self.save_cloud and self.cloud is not None:
                written.append(self._write_cloud(stem))
        except Exception as exc:                        # noqa: BLE001
            message = "failed to save map: %s" % exc
            self.get_logger().error(message)
            return False, message

        message = "map saved (%s): %s" % (reason, ", ".join(written))
        self.get_logger().info(message)
        return True, message


    def _slope_mask(self, info):
        """Cells too steep to drive, from the 3D cloud, as a 2D obstacle.

        WHY A 2D MAP NEEDS THIS

        An occupancy grid can say "something is here" and cannot say "the
        ground here tilts 30 degrees". On flat-floor robots that distinction
        never comes up. On Rubicon it is the whole problem: the terrain has 5 m
        of relief, this robot tips over past roughly 25 degrees, and a planner
        reading only the occupancy grid will happily route straight up a face
        the robot cannot hold. That is not hypothetical -- it is what happened.
        Nav2 planned a 4 m path across ground the map correctly showed as
        empty, and the robot drove into it and rolled onto its side.

        The information was never missing, only unused: it is in the 3D cloud
        the same graph produced. This takes the lowest return in each cell as
        the ground, fits the local gradient over a footprint-sized window, and
        marks anything above the limit as an obstacle.

        HOW GOOD IT IS

        Scored against the simulator's own heightmap, over every cell with a
        ground estimate: correlation 0.54, median error +1.8 deg, 90th
        percentile absolute error 21 deg. That is a rough signal, not a survey.
        The lidar returns almost nothing from ground level -- 1.6% of a scan
        lands within 0.3 m of the foot plane -- so cells are sparsely sampled
        and a single return sitting on a rock reads as terrain.

        So the threshold is CALIBRATED, not derived. Five separate runs put
        this robot on its side; scoring the resulting map at each candidate
        limit against those five sites, and against the poses the robot drove
        without falling:

          limit / window   occupied   driven path   largest    tip sites
                                        drivable    region      blocked
          none (off)          2.5%        94.9%     313 m2        0 of 5
          25 deg / 1.0 m     29.2%        67.9%      66 m2        5 of 5
          30 deg / 1.0 m     25.2%        69.2%      84 m2        3 of 5
          35 deg / 1.5 m     19.3%        69.2%     138 m2        4 of 5
          40 deg / 1.5 m     15.9%        69.2%     179 m2        4 of 5

        The top row is the case for having the layer at all: with the grid as
        RTAB-Map projects it, NOT ONE of the five places that rolled the robot
        is marked. The bottom rows are the cost of using it -- it blocks real
        ground, and the connected drivable region shrinks.

        35 deg over a 1.5 m window is the chosen trade: four of the five sites
        blocked for twice the drivable area of the strictest setting. Note that
        35 deg here is a smoothed, sparsely-sampled estimate and not the angle
        the robot tips at, which is nearer 25; the number is where it is
        because that is where it scored best, not from any model of the robot.

        slope_layer:=false turns it off and leaves the grid as RTAB-Map
        projected it.
        """
        cloud = point_cloud2.read_points(
            self.cloud, field_names=["x", "y", "z"], skip_nans=True)
        if len(cloud) < 100:
            return None, "cloud too small for a slope layer"

        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        col = np.clip(((cloud["x"] - ox) / res).astype(int), 0, w - 1)
        row = np.clip(((cloud["y"] - oy) / res).astype(int), 0, h - 1)

        # The lowest return in a cell is the ground in it. Anything above is
        # something standing on the ground, which the occupancy grid already
        # has an opinion about.
        flat = row.astype(np.int64) * w + col
        order = np.lexsort((cloud["z"], flat))
        fs = flat[order]
        first = np.empty(len(fs), dtype=bool)
        first[0] = True
        np.not_equal(fs[1:], fs[:-1], out=first[1:])
        ground = np.zeros(h * w)
        seen = np.zeros(h * w, dtype=bool)
        ground[fs[first]] = cloud["z"][order][first]
        seen[fs[first]] = True
        ground = ground.reshape(h, w)
        seen = seen.reshape(h, w)

        # Average over observed cells only. Treating unobserved ground as zero
        # would put a cliff around every hole in the coverage.
        k = max(3, int(round(self.slope_window / res)) | 1)
        total = _box_sum(np.where(seen, ground, 0.0), k)
        count = _box_sum(seen.astype(float), k)
        coverage = count / float(k * k)
        smooth = np.where(count > 0, total / np.maximum(count, 1e-9), 0.0)

        gy, gx = np.gradient(smooth, res)
        slope = np.degrees(np.arctan(np.hypot(gx, gy)))
        steep = (coverage >= self.slope_min_coverage) & (slope > self.slope_limit)

        # A steep cell is not passable at its edge either: grow it by the
        # footprint, the same way a boulder blocks more than its own outline.
        grow = max(3, int(round(0.30 / res)) | 1)
        steep = _box_sum(steep.astype(float), grow) > 0
        return steep, ("slope layer: %.1f%% of cells over %.0f deg, from %.1f%% "
                       "of cells with a ground estimate"
                       % (100.0 * steep.mean(), self.slope_limit,
                          100.0 * seen.mean()))

    def _write_grid(self, stem):
        """The nav2_map_server pair: a .pgm of the cells and a .yaml beside it."""
        info = self.grid.info
        cells = np.asarray(self.grid.data, dtype=np.int16).reshape(
            info.height, info.width)

        # nav2_map_server's "trinary" convention, which is what its loader
        # assumes unless the YAML says otherwise:
        #   254  free        cell <= 100 * free_thresh
        #     0  occupied    cell >= 100 * occupied_thresh
        #   205  unknown     everything else, including -1
        image = np.full(cells.shape, 205, dtype=np.uint8)
        image[(cells >= 0) & (cells <= 100 * self.free_thresh)] = 254
        image[cells >= 100 * self.occupied_thresh] = 0

        self.slope_note = ""
        if self.slope_layer and self.cloud is not None:
            try:
                steep, note = self._slope_mask(info)
            except Exception as exc:                    # noqa: BLE001
                steep, note = None, "slope layer failed: %s" % exc
            if steep is not None:
                image[steep] = 0
            self.slope_note = note
            self.get_logger().info(note)
        # PGM's first row is the TOP of the image, an OccupancyGrid's first row
        # is its lowest y. Without this flip the saved map comes back mirrored
        # about the x axis, which looks plausible enough to waste an afternoon.
        image = np.flipud(image)

        pgm = stem + ".pgm"
        with open(pgm, "wb") as handle:
            handle.write(b"P5\n")
            handle.write(b"# saved by magi_map_saver\n")
            handle.write(b"%d %d\n255\n" % (info.width, info.height))
            handle.write(image.tobytes())

        q = info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaml_path = stem + ".yaml"
        with open(yaml_path, "w") as handle:
            handle.write(
                "# Saved by magi_map_saver from the RTAB-Map occupancy grid.\n"
                "# `image` is deliberately relative: nav2_map_server resolves\n"
                "# it against this file's own directory, so the pair can be\n"
                "# moved or renamed together.\n"
                "image: %s\n"
                "mode: trinary\n"
                "resolution: %.6f\n"
                "origin: [%.6f, %.6f, %.6f]\n"
                "negate: 0\n"
                "occupied_thresh: %.3f\n"
                "free_thresh: %.3f\n"
                % (os.path.basename(pgm), info.resolution,
                   info.origin.position.x, info.origin.position.y, yaw,
                   self.occupied_thresh, self.free_thresh))

        known = int(np.count_nonzero(cells >= 0))
        return ("%s (%dx%d, %.1f%% known%s)"
                % (os.path.basename(yaml_path), info.width, info.height,
                   100.0 * known / max(cells.size, 1),
                   ", with slope layer" if self.slope_note else ""))

    def _write_cloud(self, stem):
        """The 3D map as a binary PLY, which every mesh viewer opens."""
        fields = {f.name: f for f in self.cloud.fields}
        wanted = ["x", "y", "z"]
        colour = "rgb" in fields
        if colour:
            wanted.append("rgb")

        points = point_cloud2.read_points(
            self.cloud, field_names=wanted, skip_nans=True)
        n = len(points)

        # Packed, explicitly little-endian, and laid out in the same order as
        # the header below, so the whole buffer goes out in one write. A
        # per-point struct.pack loop produces identical bytes and takes minutes
        # on a map this size.
        names = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        if colour:
            names += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
        out = np.empty(n, dtype=np.dtype(names))
        for axis in ("x", "y", "z"):
            out[axis] = points[axis]
        if colour:
            # rgb is conventionally carried as a float32 whose BITS are
            # 0x00RRGGBB -- reading it as a number and rounding would destroy
            # it, so reinterpret rather than convert. Some publishers use a
            # genuine uint32 for the same layout; both are handled.
            raw = np.ascontiguousarray(points["rgb"])
            packed = raw.view(np.uint32) if raw.dtype.kind == "f" else \
                raw.astype(np.uint32)
            out["red"] = (packed >> 16) & 0xFF
            out["green"] = (packed >> 8) & 0xFF
            out["blue"] = packed & 0xFF

        path = stem + ".ply"
        with open(path, "wb") as handle:
            handle.write(b"ply\nformat binary_little_endian 1.0\n")
            handle.write(b"comment saved by magi_map_saver\n")
            handle.write(b"element vertex %d\n" % n)
            handle.write(b"property float x\nproperty float y\nproperty float z\n")
            if colour:
                handle.write(b"property uchar red\nproperty uchar green\n"
                             b"property uchar blue\n")
            handle.write(b"end_header\n")
            handle.write(out.tobytes())

        return "%s (%d points)" % (os.path.basename(path), n)


def main():
    # rclpy's own signal handling is declined here, and that is the whole
    # reason shutdown saving works.
    #
    # With the default handlers, SIGINT tears the context down from inside the
    # handler while the executor is between callbacks, and the next
    # wait_for_ready_callbacks raises RCLError -- "the given context is not
    # valid" -- rather than the documented ExternalShutdownException. Nothing
    # catches it, main unwinds, and the map is never written. Observed exactly
    # once, on a real Ctrl-C, after the on-demand save had been passing all
    # along: the two paths do not share this failure.
    #
    # Taking the signals means shutdown is a flag the spin loop reads, so the
    # save runs in ordinary control flow with the context still alive.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = MapSaver()

    stop = False

    def _stop(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                            # noqa: BLE001
        # Still worth trying to save: a map in hand beats a clean traceback.
        node.get_logger().error("spin ended abnormally: %s" % exc)

    if node.save_on_shutdown:
        node.save("shutdown")

    try:
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception:                                   # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
