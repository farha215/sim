#!/usr/bin/env python3
import rclpy
import signal
import pygame
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_thrusters")

        # ================= Controller Subscriptions =================
        self.sub_front = self.create_subscription(
            Float64, 'new_thrust_front', self.cb_front, 10
        )
        self.sub_left = self.create_subscription(
            Float64, 'new_thrust_left', self.cb_left, 10
        )
        self.sub_right = self.create_subscription(
            Float64, 'new_thrust_right', self.cb_right, 10
        )

        # ================= Thruster Publishers =================
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

        # ================= Controller Values =================
        self.ctrl_values = {
            'back_propeller': 0.0,
            'left_propeller_2': 0.0,
            'right_propeller_2': 0.0,
        }

        self.manual_offsets = {k: 0.0 for k in self.pubs}
        self.max_thrust = 100.0
        self.axis_scale = 25.0  # thrust per full stick deflection

        self.timer = self.create_timer(0.05, self.publish_all)

        # ================= Pygame Init =================
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self.get_logger().info(f"Gamepad connected: {self.joy.get_name()}")

    # ================= Controller Callbacks =================
    def cb_front(self, msg):
        self.ctrl_values['back_propeller'] = msg.data

    def cb_left(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    # ================= Gamepad Read =================
    def read_gamepad(self):
        pygame.event.pump()

        # Axes (invert Y axes)
        forward = -self.joy.get_axis(1)   # Left stick Y
        yaw = self.joy.get_axis(2)        # Right stick X
        roll = self.joy.get_axis(0)       # Left stick X
        vertical = -self.joy.get_axis(3)  # Right stick Y

        self.manual_offsets['left_propeller_1'] = -forward * self.axis_scale - yaw * self.axis_scale
        self.manual_offsets['right_propeller_1'] = -forward * self.axis_scale + yaw * self.axis_scale

        if (roll < 0 or roll > 0):
            abs_thresh = abs(roll)
        else:
            abs_thresh = 0
        self.manual_offsets['front_propeller'] = vertical * self.axis_scale - abs_thresh * self.axis_scale
        self.manual_offsets['left_propeller_2'] = vertical * self.axis_scale  + roll * self.axis_scale
        self.manual_offsets['right_propeller_2'] = vertical * self.axis_scale - roll * self.axis_scale

        # Buttons
        if self.joy.get_button(0):  # A / Cross
            self.stop_all()

        if self.joy.get_button(1):  # B / Circle
            raise KeyboardInterrupt

    # ================= Publishing =================
    def publish_all(self):
        self.read_gamepad()

        for name, pub in self.pubs.items():
            ctrl = self.ctrl_values.get(name, 0.0)
            blended = clamp(ctrl + self.manual_offsets[name],
                            -self.max_thrust, self.max_thrust)

            msg = Float64()
            msg.data = float(blended)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.publish_all()


def main():
    rclpy.init()
    node = GamepadTeleop()

    def shutdown(*_):
        node.stop_all()
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
