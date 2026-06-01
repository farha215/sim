#!/usr/bin/env python3
import rclpy
import signal
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class GamepadCmdVel(Node):
    def __init__(self):
        super().__init__("gamepad_cmd_vel")

        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Scales (tune these)
        self.linear_scale = 20.0
        self.angular_scale = 20.0

        self.timer = self.create_timer(0.05, self.publish_cmd)

        # ================= Pygame Init =================
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self.get_logger().info(f"Gamepad connected: {self.joy.get_name()}")

    # ================= Gamepad Read =================
    def read_gamepad(self):
        pygame.event.pump()

        # Axes
        surge  = -self.joy.get_axis(1)   # forward/back
        lateral   =  self.joy.get_axis(0)   # left/right
        heave  = -self.joy.get_axis(3)   # up/down
        yaw    =  self.joy.get_axis(2)   # rotation

        return surge, lateral, heave, yaw

    # ================= Publish CMD VEL =================
    def publish_cmd(self):
        surge, lateral, heave, yaw = self.read_gamepad()

        msg = Twist()

        # Linear velocities
        msg.linear.x = clamp(surge * self.linear_scale, -100.0, 100.0)
        msg.linear.y = clamp(lateral  * self.linear_scale, -100.0, 100.0)
        msg.linear.z = clamp(heave * self.linear_scale, -100.0, 100.0)

        # Angular velocities
        msg.angular.z = clamp(yaw * self.angular_scale, -100.0, 100.0)

        # Optional: roll/pitch if needed
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        # Buttons
        if self.joy.get_button(0):  # A → stop
            self.stop()

        if self.joy.get_button(1):  # B → exit
            raise KeyboardInterrupt

        self.pub_cmd.publish(msg)

    def stop(self):
        msg = Twist()
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = GamepadCmdVel()

    def shutdown(*_):
        node.stop()
        rclpy.shutdown()
        pygame.quit()
        exit(0)

    signal.signal(signal.SIGINT, shutdown)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    main()