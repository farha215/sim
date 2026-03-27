#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np
import math

class AdaptiveLosController(Node):

    def __init__(self):
        super().__init__('adaptive_los_controller')

        self.dt = 0.05
        
        # --- TUNING PARAMETERS ---
        # Lookahead Bounds (In meters)
        self.declare_parameter("delta_min", 0.05)  # Tight tracking when close
        self.declare_parameter("delta_max", 0.2)  # Smooth recovery when far
        self.declare_parameter("delta_k", 3.0)    # How fast it transitions
        
        # PID Gains (Index: 0=X, 2=Z/Depth, 5=Yaw)
        self.declare_parameter("Kp", [3.0, 0.0, 3.5, 0.0, 0.0, 5.5])
        self.declare_parameter("Ki", [0.02, 0.0, 0.01, 0.0, 0.0, 0.02])
        self.declare_parameter("Kd", [0.6, 0.0, 0.8, 0.0, 0.0, 0.7])
        # -------------------------

        self.current = np.zeros(6)
        self.target = np.zeros(6)
        self.integral = np.zeros(6)
        self.prev_error = np.zeros(6)
        
        self.waypoints = []
        self.waypoint_idx = 0
        self.following_path = False

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/planned_path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current[0:3] = [p.x, p.y, p.z]
        # Yaw conversion
        self.current[5] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def path_callback(self, msg):
        self.waypoints = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        if len(self.waypoints) >= 2:
            self.waypoint_idx = 1
            self.following_path = True

    def control_loop(self):
        if not self.following_path or len(self.waypoints) <= self.waypoint_idx:
            return

        # 1. Geometry Setup
        A = np.array(self.waypoints[self.waypoint_idx - 1])
        B = np.array(self.waypoints[self.waypoint_idx])
        P = self.current[0:3]
        AB = B - A
        norm_AB = np.linalg.norm(AB)

        if norm_AB > 0:
            unit_AB = AB / norm_AB
            # Orthogonal projection to find Cross-Track Error
            AP = P - A
            t = np.clip(np.dot(AP, AB) / (norm_AB**2), 0.0, 1.0)
            P_proj = A + t * AB
            
            # Distance from path (Cross-Track Error)
            dist_vec = P - P_proj
            e_ct = np.linalg.norm(dist_vec)

            # 2. ADAPTIVE LOOKAHEAD CALCULATION
            d_min = self.get_parameter("delta_min").value
            d_max = self.get_parameter("delta_max").value
            k = self.get_parameter("delta_k").value
            
            # The Delta shifts based on how far off we are
            delta = (d_max - d_min) * math.exp(-k * e_ct) + d_min
            
            # 3. LOS TARGET
            los_target = P_proj + (unit_AB * delta)
            self.target[0:3] = los_target
            self.target[5] = math.atan2(los_target[1] - P[1], los_target[0] - P[0])

            # 4. Waypoint Advancement
            if np.linalg.norm(P - B) < 0.4 or t > 0.98:
                self.waypoint_idx += 1
                if self.waypoint_idx >= len(self.waypoints):
                    self.following_path = False

        # 5. PID & Body Frame Transform
        error = self.target - self.current
        error[5] = math.atan2(math.sin(error[5]), math.cos(error[5]))
        
        yaw = self.current[5]
        ex, ey = error[0], error[1]
        error[0] =  math.cos(yaw)*ex + math.sin(yaw)*ey
        error[1] = 0.0 # Force zero sway for stability

        # Standard PID calculation
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        tau = (np.array(self.get_parameter("Kp").value) * error) + \
              (np.array(self.get_parameter("Ki").value) * self.integral) + \
              (np.array(self.get_parameter("Kd").value) * derivative)
        self.prev_error = error

        # 6. Publish
        cmd = Twist()
        cmd.linear.x = float(np.clip(tau[0], -10.0, 10.0))
        cmd.linear.z = float(np.clip(tau[2], -5.0, 5.0))
        cmd.angular.z = float(np.clip(tau[5], -6.0, 6.0))
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(AdaptiveLosController())
    rclpy.shutdown()

if __name__ == "__main__":
    main()