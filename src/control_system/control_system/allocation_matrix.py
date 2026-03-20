import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np


class ThrusterMixer(Node):

    def __init__(self):
        super().__init__('thruster_mixer')

        # Geometry (meters)
        W = 0.35
        Lf = 0.30
        Lr = 0.30

        # Build full 6x5 configuration matrix B
        self.B = np.array([
            [-1,   -1,   0,    0,    0],     # Fx
            [0,   0,   0,    0,    0],     # Fy (no sway authority)
            [0,   0,  -1,   -1,   -1],     # Fz
            [0,   0,   W,   -W,    0],     # τx (roll)
            [0,   0,   Lf,   Lf,  -Lr],    # τy (pitch)
            [-W,  W,   0,    0,    0],     # τz (yaw)
        ], dtype=float)

        # Precompute pseudoinverse
        self.B_pinv = np.linalg.pinv(self.B)

        # Publishers
        self.pubs = {
            'back_propeller': self.create_publisher(
                Float64, '/hydrogen/back_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(
                Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(
                Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1': self.create_publisher(
                Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2': self.create_publisher(
                Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("Full 6DOF Thruster Mixer started")

    def cmd_callback(self, msg: Twist):

        # Desired wrench vector τ
        tau = np.array([
            msg.linear.x,   # Fx
            msg.linear.y,   # Fy
            msg.linear.z,   # Fz
            msg.angular.x,  # τx
            msg.angular.y,  # τy
            msg.angular.z   # τz
        ], dtype=float)

        # Compute thruster forces
        T = self.B_pinv @ tau

        # Optional saturation
        T = np.clip(T, -100.0, 100.0)

        # Publish in correct order
        self.publish('left_propeller_1',  T[0])  # LF
        self.publish('right_propeller_1', T[1])  # RF
        self.publish('left_propeller_2',  T[2])  # LV
        self.publish('right_propeller_2', T[3])  # RV
        self.publish('back_propeller',    T[4])  # BACK

    def publish(self, name, value):
        msg = Float64()
        msg.data = float(value)
        self.pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterMixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
