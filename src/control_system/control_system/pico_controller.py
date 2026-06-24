#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from auv_msgs.msg import ControlCommand


class PicoController(Node):
    """
    Three independent PIDs driven by /control_cmd setpoints and /pressure feedback.

    Inputs:
      /control_cmd  (auv_msgs/ControlCommand)
          delta_theta     yaw error    (rad)   -> angular.z
          delta_distance  surge error  (m)     -> linear.x
          target_depth    depth setpt  (m, +down)
          stop_thrusters  uint8: 1 zeroes surge/lateral/yaw, depth keeps tracking

      /pressure (std_msgs/Float32)  positive-down depth feedback

    Output:
      /cmd_vel  (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('pico_controller')

        self.dt = 0.05  # 20 Hz

        # PID gains — surge / depth / yaw
        self.declare_parameter('kp_surge', 1.5)
        self.declare_parameter('ki_surge', 0.0)
        self.declare_parameter('kd_surge', 0.2)

        self.declare_parameter('kp_depth', 60.0)
        self.declare_parameter('ki_depth', 0.50)
        self.declare_parameter('kd_depth', 0.00)

        # Reduce yaw gains so yaw control doesn't overpower surge
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.6)

        # Output saturation (N or N·m at /cmd_vel level — matches allocation_matrix scale)
        self.declare_parameter('surge_limit', 40.0)
        self.declare_parameter('depth_limit', 40.0)
        self.declare_parameter('yaw_limit', 40.0)

        # Integral anti-windup clamp
        self.declare_parameter('i_clamp', 5.0)

        # Loop / safety
        self.declare_parameter('input_timeout', 1.0)  # seconds before stopping on stale input

        # State
        self.have_setpoint = False
        self.have_depth = False
        self.delta_yaw = 0.0
        self.delta_d = 0.0
        self.target_depth = 0.0
        self.stop_thrusters = 0
        self.depth_meas = 0.0
        self.last_setpoint_time = self.get_clock().now()

        # PID Accumulators
        self.i_surge = self.i_depth = self.i_yaw = 0.0
        self.prev_e_surge = self.prev_e_depth = self.prev_e_yaw = 0.0

        # I/O
        self.create_subscription(ControlCommand, '/control_cmd', self.on_control_cmd, 10)
        self.create_subscription(Float32, '/pressure', self.on_altimeter, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('pico_controller up. Waiting for /control_cmd and /pressure ...')

    def on_control_cmd(self, msg: ControlCommand):
        self.delta_yaw = float(msg.delta_theta)
        self.delta_d = float(msg.delta_distance)
        self.target_depth = float(msg.target_depth)
        self.stop_thrusters = int(msg.stop_thrusters)
        self.last_setpoint_time = self.get_clock().now()
        self.have_setpoint = True

    def on_altimeter(self, msg: Float32):
        self.depth_meas = float(msg.data)
        self.have_depth = True

    def _pid(self, error, integ, prev_err, kp, ki, kd, i_clamp):
        integ = max(-i_clamp, min(i_clamp, integ + error * self.dt))
        deriv = (error - prev_err) / self.dt
        out = kp * error + ki * integ + kd * deriv
        return out, integ, error

    def control_loop(self):
        if not (self.have_setpoint and self.have_depth):
            return

        # Stale-input behavior: hold the last commanded depth, but zero surge & yaw
        # (same as stop_thrusters=1). target_depth is retained from the last /control_cmd message.
        timeout = self.get_parameter('input_timeout').value
        age = (self.get_clock().now() - self.last_setpoint_time).nanoseconds * 1e-9
        stale = age > timeout
        effective_stop = self.stop_thrusters != 0 or stale

        i_clamp = self.get_parameter('i_clamp').value

        # --- Depth PID (always active) ---
        # Altimeter is positive-down; cmd_vel linear.z follows Gazebo z-up (positive = surface).
        # So when we're shallower than target (meas < target_depth) we need linear.z < 0 to descend.
        e_depth = self.depth_meas - self.target_depth
        u_depth, self.i_depth, self.prev_e_depth = self._pid(
            e_depth, self.i_depth, self.prev_e_depth,
            self.get_parameter('kp_depth').value,
            self.get_parameter('ki_depth').value,
            self.get_parameter('kd_depth').value,
            i_clamp,
        )

        cmd = Twist()
        d_lim = self.get_parameter('depth_limit').value
        cmd.linear.z = float(max(-d_lim, min(d_lim, u_depth)))

        # --- Surge & yaw PIDs (gated by stop_thrusters OR stale input) ---
        if not effective_stop:
            u_surge, self.i_surge, self.prev_e_surge = self._pid(
                self.delta_d, self.i_surge, self.prev_e_surge,
                self.get_parameter('kp_surge').value,
                self.get_parameter('ki_surge').value,
                self.get_parameter('kd_surge').value,
                i_clamp
            )
            u_yaw, self.i_yaw, self.prev_e_yaw = self._pid(
                self.delta_yaw, self.i_yaw, self.prev_e_yaw,
                self.get_parameter('kp_yaw').value,
                self.get_parameter('ki_yaw').value,
                self.get_parameter('kd_yaw').value,
                i_clamp,
            )
            
            s_lim, y_lim = (self.get_parameter('surge_limit').value,
                            self.get_parameter('yaw_limit').value)
                                    
            cmd.linear.x = float(max(-s_lim, min(s_lim, u_surge)))
            cmd.linear.y = 0.0 # Force zero lateral movement (Under-actuated)
            cmd.angular.z = float(max(-y_lim, min(y_lim, u_yaw)))
        else:
            self.i_surge = self.i_yaw = 0.0
            self.prev_e_surge = self.prev_e_yaw = 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PicoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
