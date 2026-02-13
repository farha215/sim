 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # ================= Physical / tuning parameters =================
        self.dt = 0.005  # 200 Hz
        self.alpha = 0.98  # complementary filter

        # Cascaded control gains
        self.Kp = np.diag([5.0, 5.0])   # outer PI proportional
        self.Ki = np.diag([1.0, 1.0])   # outer PI integral

        # Inner LQR gain (angular velocity -> thrusters directly)
        # Shape: 3 thrusters x 2 angular velocities
        self.K_lqr = np.array([
            [10.0, 0.0],
            [0.0, 10.0],
            [5.0, 5.0]
        ])

        # Trim forces for hover
        self.F_trim = np.array([0.0, 0.0, 10.0])
        self.max_thrust = 40.0

        # Orientation / state
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.int_phi = 0.0
        self.int_theta = 0.0
        self.p = 0.0
        self.q = 0.0

        # Desired orientation
        self.phi_des = 0.0
        self.theta_des = 0.0

        # Latest IMU readings
        self.latest_imu = None

        # ROS subscriptions
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # Thruster publishers
        self.thrusters = [
            self.create_publisher(Float64, 'new_thrust_front', 10),
            self.create_publisher(Float64, 'new_thrust_left', 10),
            self.create_publisher(Float64, 'new_thrust_right', 10),
        ]

        # Fixed-rate control loop timer (200 Hz)
        self.create_timer(self.dt, self.control_loop)

    # ================= IMU callback =================
    def imu_cb(self, msg: Imu):
        self.latest_imu = msg

    # ================= Main control loop =================
    def control_loop(self):
        if self.latest_imu is None:
            return  

        imu = self.latest_imu

        # Extract measurements
        ax, ay, az = imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z
        p, q, r = imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z

        # Complementary filter for orientation
        phi_acc = np.arctan2(ay, az)
        theta_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        self.phi = self.alpha*(self.phi + p*self.dt) + (1 - self.alpha)*phi_acc
        self.theta = self.alpha*(self.theta + q*self.dt) + (1 - self.alpha)*theta_acc
        self.psi += r*self.dt

        self.p = p
        self.q = q

        # ================= Outer PI loop (orientation -> reference angular velocities) =================
        e_phi = self.phi_des - self.phi
        e_theta = self.theta_des - self.theta

        self.int_phi += e_phi * self.dt
        self.int_theta += e_theta * self.dt

        omega_ref = np.array([
            self.Kp[0,0]*e_phi + self.Ki[0,0]*self.int_phi,
            self.Kp[1,1]*e_theta + self.Ki[1,1]*self.int_theta
        ])

        # ================= Inner LQR loop (angular velocity error -> thruster commands) =================
        omega = np.array([self.p, self.q])
        omega_error = omega - omega_ref

        # Compute thruster commands directly
        thrusts = self.F_trim - self.K_lqr @ omega_error
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)

        # ================= Publish thruster commands =================
        for i, pub in enumerate(self.thrusters):
            msg_out = Float64()
            msg_out.data = float(thrusts[i])
            pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
