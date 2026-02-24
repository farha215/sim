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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')

        # ================= Parameters =================
        self.voxel_size = 0.2
        self.robot_radius = 0.3
        self.inflate_voxels = 0  # DISABLED FOR DEBUGGING

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
        self.get_logger().info("CLOUD CALLBACK TRIGGERED")
        self.cloud_msg = msg

    # =====================================================
    # Service Callback
    # =====================================================

    def plan_path_callback(self, request, response):

        if self.current_pose is None:
            response.success = False
            response.message = "No odom received"
            return response

        if self.cloud_msg is None:
            response.success = False
            response.message = "No cloud_map received"
            return response

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except TransformException as ex:
            response.success = False
            response.message = f"TF error: {str(ex)}"
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

        if occupancy is None:
            response.success = False
            response.message = "Voxel grid build failed"
            return response

        start_idx = self.world_to_grid(start, origin)
        goal_idx = self.world_to_grid(goal, origin)

        self.get_logger().info(f"Grid shape: {occupancy.shape}")
        self.get_logger().info(f"Start idx: {start_idx}")
        self.get_logger().info(f"Goal idx: {goal_idx}")

        # Bounds check
        if not self.in_bounds(start_idx, occupancy):
            response.success = False
            response.message = "Start outside grid bounds"
            return response

        if not self.in_bounds(goal_idx, occupancy):
            response.success = False
            response.message = "Goal outside grid bounds"
            return response

        self.get_logger().info(f"Start occupied? {occupancy[start_idx]}")
        self.get_logger().info(f"Goal occupied? {occupancy[goal_idx]}")

        occupied_ratio = np.sum(occupancy) / occupancy.size
        self.get_logger().info(f"Occupied ratio: {occupied_ratio:.3f}")

        # Force start & goal free (debug safety)
        occupancy[start_idx] = 0
        occupancy[goal_idx] = 0

        path_idx = self.astar_3d(occupancy, start_idx, goal_idx)

        if path_idx is None:
            response.success = False
            response.message = "No path found"
            return response

        path_msg = self.build_path_msg(path_idx, origin, request.target)

        self.path_pub.publish(path_msg)

        response.trajectory = path_msg
        response.success = True
        response.message = "Path planned successfully"

        return response

    # =====================================================
    # Voxel Grid Builder
    # =====================================================

    def build_voxel_grid(self):

        points = []

        for i in range(0, len(self.cloud_msg.data), self.cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', self.cloud_msg.data, offset=i)

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            points.append((x, y, z))

        if len(points) == 0:
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

        return occupancy, (min_x, min_y, min_z)

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
            (1,0,0),(-1,0,0),
            (0,1,0),(0,-1,0),
            (0,0,1),(0,0,-1)
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

                tentative = g_score[current] + 1

                if nxt not in g_score or tentative < g_score[nxt]:
                    came_from[nxt] = current
                    g_score[nxt] = tentative
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))

        return None

    # =====================================================
    # Helpers
    # =====================================================

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

    def build_path_msg(self, path_idx, origin, target_pose):

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(path_idx)):

            world = self.grid_to_world(path_idx[i], origin)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world[0]
            pose.pose.position.y = world[1]
            pose.pose.position.z = world[2]

            if i < len(path_idx) - 1:
                nxt = self.grid_to_world(path_idx[i+1], origin)

                dx = nxt[0] - world[0]
                dy = nxt[1] - world[1]
                dz = nxt[2] - world[2]

                yaw = math.atan2(dy, dx)
                pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))
                roll = 0.0

                r = R.from_euler('xyz', [roll, pitch, yaw])
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


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()