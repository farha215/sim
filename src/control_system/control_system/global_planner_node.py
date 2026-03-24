#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import heapq
import math
import struct

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from custom_interfaces.srv import PlanPath

import tf2_ros
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')

        # ================= Parameters =================
        self.voxel_size        = 0.2
        self.robot_radius      = 0.5
        self.safety_margin     = 0.1
        self.inflate_voxels    = int(math.ceil(
            (self.robot_radius + self.safety_margin) / self.voxel_size
        ))

        # --- LOS / path parameters ---
        self.resample_step     = 0.3    # [m]  arc-length resampling resolution
        self.lookahead_base    = 1.5    # [m]  base lookahead distance
        self.lookahead_k       = 0.5    # [s]  speed-scaling factor  (Δ = base + k·v)
        self.switch_threshold  = 0.4    # [m]  waypoint-switch proximity
        self.max_curvature     = 0.4    # [1/m] 1/R_min  (R_min ≈ 2.5 m)
        self.heading_window    = 5      # points for moving-average heading filter
        self.collinear_tol     = 0.02   # [rad] angle tolerance for pruning

        # ================= Internal State =================
        self.current_pose  = None
        self.cloud_msg     = None
        self.current_speed = 0.0       # updated from odom for speed-aware lookahead

        # ================= Subscribers =================
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=1)
        qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(PointCloud2, '/cloud_map', self.cloud_callback, qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ================= TF =================
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ================= Service =================
        self.srv = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)

        # ================= Publisher =================
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info("Global Planner Ready")

    # =====================================================
    # Callbacks
    # =====================================================

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        v = msg.twist.twist.linear
        self.current_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def cloud_callback(self, msg):
        self.cloud_msg = msg

    def goal_callback(self, msg):
        self.get_logger().info("Received goal from RViz")
        request  = PlanPath.Request()
        request.target = msg.pose
        response = self.plan_path_callback(request, PlanPath.Response())
        if response.success:
            self.get_logger().info("Path planned successfully")
        else:
            self.get_logger().warn(response.message)

    # =====================================================
    # Service Callback
    # =====================================================

    def plan_path_callback(self, request, response):

        if self.current_pose is None or self.cloud_msg is None:
            response.success = False
            response.message = "Missing odom or cloud"
            return response

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except TransformException as ex:
            response.success = False
            response.message = str(ex)
            return response

        start = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        goal = (
            request.target.position.x,
            request.target.position.y,
            request.target.position.z,
        )

        occupancy, origin = self.build_voxel_grid()
        if occupancy is None:
            response.success = False
            response.message = "Empty point cloud"
            return response

        start_idx = self.world_to_grid(start, origin)
        goal_idx  = self.world_to_grid(goal, origin)

        if not self.in_bounds(start_idx, occupancy) or \
           not self.in_bounds(goal_idx, occupancy):
            response.success = False
            response.message = "Start or goal outside grid"
            return response

        occupancy[start_idx] = 0
        occupancy[goal_idx]  = 0

        path_idx = self.astar_3d(occupancy, start_idx, goal_idx)
        if path_idx is None:
            response.success = False
            response.message = "No path found"
            return response

        # ── Tier 3: prune collinear nodes before smoothing ──────────────
        path_idx = self.prune_collinear(path_idx)

        # ── Tier 1: spline smooth (with overshoot guard) ─────────────────
        smoothed = self.smooth_path_spline(path_idx, origin)

        # ── Tier 1: arc-length resample → uniform spacing ────────────────
        smoothed = self.resample_by_distance(smoothed, step=self.resample_step)

        # ── Tier 2: curvature limiting ────────────────────────────────────
        smoothed = self.limit_curvature(smoothed, self.max_curvature)

        # ── Tier 3: path validity check against voxel grid ───────────────
        if not self.validate_path(smoothed, occupancy, origin):
            self.get_logger().warn(
                "Smoothed path collides — falling back to raw A* waypoints"
            )
            smoothed = [self.grid_to_world(idx, origin) for idx in path_idx]
            smoothed = self.resample_by_distance(smoothed, step=self.resample_step)

        # ── Tier 2: smooth headings (moving-average) ─────────────────────
        path_msg = self.build_path_from_world(smoothed, request.target)
        self.path_pub.publish(path_msg)

        response.trajectory = path_msg
        response.success     = True
        response.message     = "Path planned successfully"
        return response

    # =====================================================
    # Voxel Grid
    # =====================================================

    def build_voxel_grid(self):

        points = []
        for i in range(0, len(self.cloud_msg.data), self.cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', self.cloud_msg.data, offset=i)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append((x, y, z))

        if not points:
            return None, None

        points = np.array(points)
        min_x, min_y, min_z = points.min(axis=0)
        max_x, max_y, max_z = points.max(axis=0)

        size_x = int((max_x - min_x) / self.voxel_size) + 1
        size_y = int((max_y - min_y) / self.voxel_size) + 1
        size_z = int((max_z - min_z) / self.voxel_size) + 1

        occupancy = np.zeros((size_x, size_y, size_z), dtype=np.uint8)
        for p in points:
            i = int((p[0] - min_x) / self.voxel_size)
            j = int((p[1] - min_y) / self.voxel_size)
            k = int((p[2] - min_z) / self.voxel_size)
            occupancy[i, j, k] = 1

        # Spherical inflation
        inflated = occupancy.copy()
        r2 = self.inflate_voxels ** 2
        for i in range(size_x):
            for j in range(size_y):
                for k in range(size_z):
                    if occupancy[i, j, k] == 1:
                        for dx in range(-self.inflate_voxels, self.inflate_voxels + 1):
                            for dy in range(-self.inflate_voxels, self.inflate_voxels + 1):
                                for dz in range(-self.inflate_voxels, self.inflate_voxels + 1):
                                    if dx*dx + dy*dy + dz*dz > r2:
                                        continue
                                    ni, nj, nk = i+dx, j+dy, k+dz
                                    if 0 <= ni < size_x and \
                                       0 <= nj < size_y and \
                                       0 <= nk < size_z:
                                        inflated[ni, nj, nk] = 1

        return inflated, (min_x, min_y, min_z)

    # =====================================================
    # A* 3D
    # =====================================================

    def astar_3d(self, grid, start, goal):

        def heuristic(a, b):
            return math.sqrt(
                (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2
            )

        neighbors = [
            (dx, dy, dz)
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
            for dz in [-1, 0, 1]
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy, dz in neighbors:
                nxt = (current[0]+dx, current[1]+dy, current[2]+dz)

                if not self.in_bounds(nxt, grid) or grid[nxt] == 1:
                    continue

                step_cost = math.sqrt(dx*dx + dy*dy + dz*dz)
                tentative  = g_score[current] + step_cost

                if nxt not in g_score or tentative < g_score[nxt]:
                    came_from[nxt] = current
                    g_score[nxt]   = tentative
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))

        return None

    # =====================================================
    # Tier 3 — Collinear Pruning
    # =====================================================

    def prune_collinear(self, path_idx):
        """Remove intermediate grid nodes that are nearly collinear.

        Reduces A* zig-zag artefacts before smoothing, producing a cleaner
        spline with less computation.
        """
        if len(path_idx) <= 2:
            return path_idx

        pruned = [path_idx[0]]
        for i in range(1, len(path_idx) - 1):
            a = np.array(pruned[-1],    dtype=float)
            b = np.array(path_idx[i],  dtype=float)
            c = np.array(path_idx[i+1], dtype=float)

            ab = b - a
            bc = c - b
            n_ab = np.linalg.norm(ab)
            n_bc = np.linalg.norm(bc)

            if n_ab < 1e-9 or n_bc < 1e-9:
                continue  # degenerate — drop

            cos_angle = np.dot(ab, bc) / (n_ab * n_bc)
            angle = math.acos(max(-1.0, min(1.0, cos_angle)))

            if angle > self.collinear_tol:
                pruned.append(path_idx[i])

        pruned.append(path_idx[-1])
        return pruned

    # =====================================================
    # Tier 1 — Spline Smooth (overshoot-guarded)
    # =====================================================

    def smooth_path_spline(self, path_idx, origin, num_points=300):
        """Fit a B-spline through grid waypoints.

        Uses a modest smoothing factor (s > 0) to avoid oscillation at sharp
        turns.  Falls back to raw world points if the spline fails.
        """
        world_points = [self.grid_to_world(idx, origin) for idx in path_idx]

        if len(world_points) < 4:
            return world_points

        x = [p[0] for p in world_points]
        y = [p[1] for p in world_points]
        z = [p[2] for p in world_points]

        try:
            # s > 0 prevents exact interpolation → dampens overshoots near
            # tight corners.  k=3 (cubic) is standard.
            tck, _ = splprep([x, y, z], s=1.0, k=3)
            u_fine = np.linspace(0, 1, num_points)
            xs, ys, zs = splev(u_fine, tck)
            return list(zip(xs, ys, zs))
        except Exception as e:
            self.get_logger().warn(f"Spline failed ({e}), using raw path")
            return world_points

    # =====================================================
    # Tier 1 — Arc-Length Resampling  (NON-NEGOTIABLE)
    # =====================================================

    def resample_by_distance(self, points, step=0.3):
        """Resample *points* at constant arc-length intervals of *step* metres.

        This is the single most important fix for LOS guidance: the lookahead
        circle intersection and cross-track projection both assume uniform
        spatial spacing.  Parameterising the spline in *u* (not arc-length)
        creates variable spacing that breaks LOS consistency.

        Returns a new list of (x, y, z) tuples with ~step-metre spacing.
        """
        if len(points) < 2:
            return list(points)

        pts = np.array(points, dtype=float)

        # Cumulative arc-length along the path
        deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        arc    = np.concatenate([[0.0], np.cumsum(deltas)])
        total  = arc[-1]

        if total < step:
            return list(points)           # path shorter than one step

        # Desired sample positions along arc
        s_samples = np.arange(0.0, total, step)
        if s_samples[-1] < total:
            s_samples = np.append(s_samples, total)  # always include the end

        # Linear interpolation for each axis
        xs = np.interp(s_samples, arc, pts[:, 0])
        ys = np.interp(s_samples, arc, pts[:, 1])
        zs = np.interp(s_samples, arc, pts[:, 2])

        return list(zip(xs, ys, zs))

    # =====================================================
    # Tier 2 — Curvature Limiting
    # =====================================================

    def limit_curvature(self, points, kappa_max):
        """Iteratively smooth the path so that curvature ≤ kappa_max everywhere.

        Discrete curvature at waypoint i is estimated as the inverse of the
        circumradius of the triangle (i-1, i, i+1).  If it exceeds kappa_max,
        point i is moved towards the midpoint of its neighbours until the
        constraint is satisfied (or the loop exhausts).

        This makes the path kinematically feasible for the AUV's minimum
        turning radius  R_min = 1/kappa_max.
        """
        if len(points) < 3 or kappa_max <= 0:
            return points

        pts = np.array(points, dtype=float)

        for _ in range(50):           # iterate until convergence
            changed = False
            for i in range(1, len(pts) - 1):
                a, b, c = pts[i-1], pts[i], pts[i+1]
                kappa = self._circumcurvature(a, b, c)

                if kappa > kappa_max:
                    # Blend point towards its neighbours' midpoint
                    alpha = 0.3
                    mid   = 0.5 * (a + c)
                    pts[i] = (1 - alpha) * b + alpha * mid
                    changed = True

            if not changed:
                break

        return [tuple(p) for p in pts]

    @staticmethod
    def _circumcurvature(a, b, c):
        """Curvature at *b* via circumscribed-circle formula."""
        ab = np.linalg.norm(b - a)
        bc = np.linalg.norm(c - b)
        ac = np.linalg.norm(c - a)

        if ab < 1e-9 or bc < 1e-9 or ac < 1e-9:
            return 0.0

        # Area of triangle via cross product
        cross = np.cross(b - a, c - a)
        area  = 0.5 * np.linalg.norm(cross)

        if area < 1e-9:
            return 0.0                 # collinear — zero curvature

        # R = (ab · bc · ac) / (4 · area)
        R = (ab * bc * ac) / (4.0 * area)
        return 1.0 / R

    # =====================================================
    # Tier 3 — Path Validity Check
    # =====================================================

    def validate_path(self, points, occupancy, origin):
        """Return True if every waypoint in *points* is free in the voxel grid.

        Catches cases where spline smoothing cuts corners back into occupied
        cells, preventing silent failures downstream.
        """
        for p in points:
            idx = self.world_to_grid(p, origin)
            if not self.in_bounds(idx, occupancy):
                return False
            if occupancy[idx] == 1:
                return False
        return True

    # =====================================================
    # Heading Computation — Tier 2 (smoothed)
    # =====================================================

    def _compute_headings(self, points):
        """Return per-point (yaw, pitch) using a moving-average window.

        Raw finite-difference headings are noisy because the discrete spacing
        amplifies small positional errors.  Averaging over *heading_window*
        consecutive tangent vectors kills the jitter before it reaches the
        PID / LOS controller.

        Returns list of (yaw_rad, pitch_rad) — one entry per point.
        """
        n   = len(points)
        pts = np.array(points, dtype=float)
        w   = self.heading_window

        # Raw tangent at each point (central difference where possible)
        raw_yaw   = []
        raw_pitch = []
        for i in range(n):
            i0 = max(0, i - 1)
            i1 = min(n - 1, i + 1)
            d  = pts[i1] - pts[i0]
            dx, dy, dz = d
            raw_yaw.append(math.atan2(dy, dx))
            raw_pitch.append(math.atan2(-dz, math.sqrt(dx**2 + dy**2)))

        # Smooth via sliding window on sin/cos to avoid wrap-around issues
        def smooth_angles(angles):
            sins = np.sin(angles)
            coss = np.cos(angles)
            kernel = np.ones(w) / w
            s_sm = np.convolve(sins, kernel, mode='same')
            c_sm = np.convolve(coss, kernel, mode='same')
            return [math.atan2(s, c) for s, c in zip(s_sm, c_sm)]

        smooth_yaw   = smooth_angles(raw_yaw)
        smooth_pitch = smooth_angles(raw_pitch)
        return list(zip(smooth_yaw, smooth_pitch))

    # =====================================================
    # LOS Helpers — Tier 1 / 2
    # =====================================================

    def lookahead_distance(self):
        """Speed-adaptive lookahead: Δ = base + k · v.

        At low speeds the lookahead shrinks → precise tracking.
        At high speeds it grows → smoother, less oscillation.
        """
        return self.lookahead_base + self.lookahead_k * self.current_speed

    def find_los_target(self, path_points, robot_pos):
        """Return the LOS target point and active segment index.

        Algorithm (segment-based, Tier 1 fix):
        1. Find the path point nearest to *robot_pos*.
        2. Walk forward along the segment list until the cumulative arc-length
           exceeds the lookahead distance Δ.
        3. Interpolate the exact lookahead point on that segment.

        Returns (target_point, segment_idx) or (None, None) if the robot is
        past the end.
        """
        if not path_points:
            return None, None

        pts    = np.array(path_points, dtype=float)
        robot  = np.array(robot_pos,   dtype=float)
        delta  = self.lookahead_distance()

        # Nearest point index
        dists = np.linalg.norm(pts - robot, axis=1)
        near  = int(np.argmin(dists))

        # Walk forward accumulating arc-length
        accumulated = 0.0
        for i in range(near, len(pts) - 1):
            seg_len = np.linalg.norm(pts[i+1] - pts[i])

            if accumulated + seg_len >= delta:
                # Interpolate within this segment
                remain = delta - accumulated
                t      = remain / seg_len if seg_len > 1e-9 else 0.0
                target = pts[i] + t * (pts[i+1] - pts[i])
                return tuple(target), i

            accumulated += seg_len

        # Past the end of the path
        return tuple(pts[-1]), len(pts) - 2

    def should_switch_segment(self, path_points, robot_pos, seg_idx):
        """Return True when the robot should advance to segment seg_idx+1.

        Switches when the robot has passed the next waypoint or is within
        *switch_threshold* metres of it — prevents oscillation at segment
        junctions.
        """
        if seg_idx + 1 >= len(path_points):
            return False

        next_wp = np.array(path_points[seg_idx + 1], dtype=float)
        robot   = np.array(robot_pos, dtype=float)
        return float(np.linalg.norm(next_wp - robot)) < self.switch_threshold

    # =====================================================
    # Build Path Message
    # =====================================================

    def build_path_from_world(self, world_points, target_pose):
        """Assemble a nav_msgs/Path from world-frame waypoints.

        Orientations are computed from the *smoothed* heading (yaw + pitch),
        not raw finite differences, so the published orientations are suitable
        for direct use by a LOS / PID controller.
        """
        path        = Path()
        path.header.frame_id = "map"
        path.header.stamp    = self.get_clock().now().to_msg()

        headings = self._compute_headings(world_points)

        for i, (wp, (yaw, pitch)) in enumerate(zip(world_points, headings)):

            pose             = PoseStamped()
            pose.header      = path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]

            if i < len(world_points) - 1:
                rot = R.from_euler('xyz', [0.0, pitch, yaw])
                q   = rot.as_quat()          # (x, y, z, w)
            else:
                q = (
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w,
                )

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)

        return path

    # =====================================================
    # Grid Utilities
    # =====================================================

    def in_bounds(self, idx, grid):
        return (0 <= idx[0] < grid.shape[0] and
                0 <= idx[1] < grid.shape[1] and
                0 <= idx[2] < grid.shape[2])

    def world_to_grid(self, world, origin):
        return (
            int((world[0] - origin[0]) / self.voxel_size),
            int((world[1] - origin[1]) / self.voxel_size),
            int((world[2] - origin[2]) / self.voxel_size),
        )

    def grid_to_world(self, idx, origin):
        return (
            origin[0] + idx[0] * self.voxel_size,
            origin[1] + idx[1] * self.voxel_size,
            origin[2] + idx[2] * self.voxel_size,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()