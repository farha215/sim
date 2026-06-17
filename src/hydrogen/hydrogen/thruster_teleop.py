#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from auv_msgs.msg import ControlCommand


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_thrusters")

        self.pub_to_pico = self.create_publisher(ControlCommand, '/control_cmd', 10)

        self.surge_scale  = 5.0
        self.yaw_scale    = 1.5
        self.surge_step   = 0.5    # delta_d per key press
        self.yaw_step     = 0.15   # delta_yaw per key press
        self.depth_step   = 0.1    # m per arrow press
        self.depth_min    = 0.0
        self.depth_max    = 10.0

        self.delta_d      = 0.0
        self.delta_yaw    = 0.0
        self.target_depth = 0.0
        self.stop_bit     = 0

        self.timer = self.create_timer(0.05, self.publish_cmd)

    def publish_cmd(self):
        msg = ControlCommand()
        msg.delta_distance = float(self.delta_d)
        msg.delta_theta    = float(self.delta_yaw)
        msg.target_depth   = float(self.target_depth)
        msg.stop_thrusters = self.stop_bit
        self.pub_to_pico.publish(msg)

    def stop(self):
        self.delta_d   = 0.0
        self.delta_yaw = 0.0
        self.stop_bit  = 1
        self.publish_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
Keyboard Teleop → /control_cmd

  W / S           : Surge forward / backward
  A / D           : Yaw left / right
  Arrow Up / Down : Target depth shallower / deeper
  SPACE           : Zero surge & yaw (hold depth)
  X               : Exit
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[A':   # up → shallower
                        node.target_depth -= node.depth_step
                        node.target_depth = clamp(node.target_depth, node.depth_min, node.depth_max)
                        node.get_logger().info(f"target_depth = {node.target_depth:.2f} m")

                    elif seq == '[B': # down → deeper
                        node.target_depth += node.depth_step
                        node.target_depth = clamp(node.target_depth, node.depth_min, node.depth_max)
                        node.get_logger().info(f"target_depth = {node.target_depth:.2f} m")

                else:
                    ch = ch.lower()

                    if ch == 'w':
                        node.delta_d += node.surge_step
                    elif ch == 's':
                        node.delta_d -= node.surge_step
                    elif ch == 'a':
                        node.delta_yaw += node.yaw_step
                    elif ch == 'd':
                        node.delta_yaw -= node.yaw_step
                    elif ch == ' ':
                        node.delta_d   = 0.0
                        node.delta_yaw = 0.0
                        node.stop_bit  = 1
                    elif ch == 'x':
                        exit_clean()

                # Clamp values
                node.delta_d   = clamp(node.delta_d,   -node.surge_scale, node.surge_scale)
                node.delta_yaw = clamp(node.delta_yaw, -node.yaw_scale,   node.yaw_scale)

                # Clear stop_bit once a non-space key was pressed
                if ch != ' ':
                    node.stop_bit = 0

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()
