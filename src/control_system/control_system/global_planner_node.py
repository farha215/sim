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
        self.voxel_size = 0.2
        self.robot_radius = 0.5
        self.safety_margin = 0.1
        self.inflate_voxels = int(math.ceil(
            (self.robot_radius + self.safety_margin) / self.voxel_size
        ))

        # ================= Internal State =================
        self.current_pose = None
        self.cloud_msg = None

        # ================= Subscribers =================
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(
            PointCloud2,
            '/cloud_map',
            self.cloud_callback,
            qos
        )

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # ================= TF =================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ================= Service =================
        self.srv = self.create_service(
            PlanPath,
            'plan_path',
            self.plan_path_callback
        )

        # ================= Publisher =================
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info("Global Planner Ready")

    # =====================================================
    # Callbacks
    # =====================================================

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def cloud_callback(self, msg):
        self.cloud_msg = msg

    def goal_callback(self, msg):
        self.get_logger().info("Received goal from RViz")
        request = PlanPath.Request()
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
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except TransformException as ex:
            response.success = False
            response.message = str(ex)
            return response

        start = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )

        goal = (
            request.target.position.x,
            request.target.position.y,
            request.target.position.z
        )

        occupancy, origin = self.build_voxel_grid()

        start_idx = self.world_to_grid(start, origin)
        goal_idx = self.world_to_grid(goal, origin)

        if not self.in_bounds(start_idx, occupancy) or \
           not self.in_bounds(goal_idx, occupancy):

            response.success = False
            response.message = "Start or goal outside grid"
            return response

        occupancy[start_idx] = 0
        occupancy[goal_idx] = 0

        path_idx = self.astar_3d(occupancy, start_idx, goal_idx)

        if path_idx is None:
            response.success = False
            response.message = "No path found"
            return response

        # ===== Smooth Path =====
        smoothed_world = self.smooth_path_spline(path_idx, origin)

        path_msg = self.build_path_from_world(smoothed_world, request.target)
        self.path_pub.publish(path_msg)

        response.trajectory = path_msg
        response.success = True
        response.message = "Path planned successfully"
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

        # ===== Spherical Inflation =====
        inflated = occupancy.copy()

        for i in range(size_x):
            for j in range(size_y):
                for k in range(size_z):
                    if occupancy[i, j, k] == 1:
                        for dx in range(-self.inflate_voxels, self.inflate_voxels + 1):
                            for dy in range(-self.inflate_voxels, self.inflate_voxels + 1):
                                for dz in range(-self.inflate_voxels, self.inflate_voxels + 1):

                                    if dx*dx + dy*dy + dz*dz > self.inflate_voxels*self.inflate_voxels:
                                        continue

                                    ni = i + dx
                                    nj = j + dy
                                    nk = k + dz

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
                (a[0]-b[0])**2 +
                (a[1]-b[1])**2 +
                (a[2]-b[2])**2
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

                if not self.in_bounds(nxt, grid):
                    continue

                if grid[nxt] == 1:
                    continue

                step_cost = math.sqrt(dx*dx + dy*dy + dz*dz)
                tentative = g_score[current] + step_cost

                if nxt not in g_score or tentative < g_score[nxt]:
                    came_from[nxt] = current
                    g_score[nxt] = tentative
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))

        return None

    # =====================================================
    # Helpers
    # =====================================================

    def smooth_path_spline(self, path_idx, origin, num_points=100):

        world_points = [self.grid_to_world(idx, origin) for idx in path_idx]

        if len(world_points) < 4:
            return world_points

        x = [p[0] for p in world_points]
        y = [p[1] for p in world_points]
        z = [p[2] for p in world_points]

        try:
            tck, u = splprep([x, y, z], s=0.5)
            u_fine = np.linspace(0, 1, num_points)
            x_fine, y_fine, z_fine = splev(u_fine, tck)
            return list(zip(x_fine, y_fine, z_fine))
        except:
            return world_points

    def build_path_from_world(self, world_points, target_pose):

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(world_points)):

            pose = PoseStamped()
            pose.header = path.header

            pose.pose.position.x = world_points[i][0]
            pose.pose.position.y = world_points[i][1]
            pose.pose.position.z = world_points[i][2]

            if i < len(world_points) - 1:
                dx = world_points[i+1][0] - world_points[i][0]
                dy = world_points[i+1][1] - world_points[i][1]
                dz = world_points[i+1][2] - world_points[i][2]

                yaw = math.atan2(dy, dx)
                pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))
                r = R.from_euler('xyz', [0.0, pitch, yaw])
                q = r.as_quat()
            else:
                q = (
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                )

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)

        return path

    def in_bounds(self, idx, grid):
        return (0 <= idx[0] < grid.shape[0] and
                0 <= idx[1] < grid.shape[1] and
                0 <= idx[2] < grid.shape[2])

    def world_to_grid(self, world, origin):
        return (
            int((world[0] - origin[0]) / self.voxel_size),
            int((world[1] - origin[1]) / self.voxel_size),
            int((world[2] - origin[2]) / self.voxel_size)
        )

    def grid_to_world(self, idx, origin):
        return (
            origin[0] + idx[0] * self.voxel_size,
            origin[1] + idx[1] * self.voxel_size,
            origin[2] + idx[2] * self.voxel_size
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()