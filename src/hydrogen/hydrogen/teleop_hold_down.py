#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_thrusters")

        # ================= Controller Subscriptions (ONLY 3 thrusters) =================
        self.sub_front = self.create_subscription(
            Float64, 'new_thrust_front', self.cb_front, 10
        )
        self.sub_left = self.create_subscription(
            Float64, 'new_thrust_left', self.cb_left, 10
        )
        self.sub_right = self.create_subscription(
            Float64, 'new_thrust_right', self.cb_right, 10
        )

        # ================= Thruster Command Publishers (ALL 5) =================
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

        # ================= Controller Values (ONLY 3 USED) =================
        self.ctrl_values = {
            'back_propeller': 0.0,
            'left_propeller_2': 0.0,
            'right_propeller_2': 0.0,
        }

        # ================= Manual Offsets for X/Y motion (reset each cycle) =================
        self.manual_offsets = {k: 0.0 for k in self.pubs.keys()}

        self.step = 50.0
        self.max_thrust = 50.0

        # ================= Vertical Toggle State (PERSISTENT) =================
        self.vertical_thrust = 0.0
        self.vertical_step = 5.0
        self.vertical_max = 100.0

        self.timer = self.create_timer(0.02, self.publish_all)

    # ================= Controller Callbacks =================
    def cb_front(self, msg):
        self.ctrl_values['back_propeller'] = msg.data

    def cb_left(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    # ================= Publishing =================
    def publish_all(self):
        for name, pub in self.pubs.items():
            ctrl = self.ctrl_values.get(name, 0.0)


            # Vertical thrusters get the persistent toggle value on top
            if name in ('left_propeller_2', 'right_propeller_2'):
                blended = ctrl + self.manual_offsets[name] + self.vertical_thrust # small boost to help with weight
            
            elif name == 'back_propeller':
                blended = ctrl + self.manual_offsets[name] + (self.vertical_thrust * 0.83)

            else:
                blended = ctrl + self.manual_offsets[name]
           

            """
            if name == 'back_propeller':                             # rear vertical — 2x to balance
                blended = ctrl + self.manual_offsets[name] + (self.vertical_thrust)
            elif name in ('left_propeller_2', 'right_propeller_2'): # front vertical — 1x
                blended = ctrl + self.manual_offsets[name] + self.vertical_thrust
            else:                                                    # horizontal — no vertical
                blended = ctrl + self.manual_offsets[name]

            blended = clamp(blended, -self.vertical_max, self.vertical_max)
            """

            msg = Float64()
            msg.data = float(blended)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.vertical_thrust = 0.0
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
Teleop + Controller Blending Active

Controller + Keyboard:
  - back_propeller
  - left_propeller_2
  - right_propeller_2

Keyboard ONLY:
  - left_propeller_1
  - right_propeller_1

Controls:
  A / D           : Ascend / Descend (TOGGLE — persists after release)
  Arrow Left/Right: Yaw
  W / S           : Forward / Backward
  I / K           : Pitch
  l / r           : Roll left and right
  SPACE           : Clear ALL offsets (including vertical)
  X               : Exit
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():

            for k in node.manual_offsets:
                node.manual_offsets[k] = 0.0

            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[C':  # yaw right
                        node.manual_offsets['left_propeller_1'] = node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif seq == '[D':  # yaw left
                        node.manual_offsets['left_propeller_1'] = -node.step
                        node.manual_offsets['right_propeller_1'] = node.step

                else:
                    ch = ch.lower()

                    if ch == 'w':  # forward
                        node.manual_offsets['left_propeller_1'] = -node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif ch == 'a':  # ascend
                        node.vertical_thrust = clamp(
                            node.vertical_thrust + node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Vertical thrust: {node.vertical_thrust:+.1f}  |  back: {node.vertical_thrust*2.0:+.1f}")

                    elif ch == 'd':  # descend
                        node.vertical_thrust = clamp(
                            node.vertical_thrust - node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Vertical thrust: {node.vertical_thrust:+.1f}  |  back: {node.vertical_thrust*2.0:+.1f}")

                    elif ch == 's':  # backward
                        node.manual_offsets['left_propeller_1'] = node.step
                        node.manual_offsets['right_propeller_1'] = node.step

                    elif ch == 'i':  # pitch down
                        node.manual_offsets['left_propeller_2'] = node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == 'k':  # pitch up
                        node.manual_offsets['left_propeller_2'] = -node.step
                        node.manual_offsets['right_propeller_2'] = node.step

                    elif ch == 'l':  # roll left
                        node.manual_offsets['left_propeller_2'] = -node.step
                        node.manual_offsets['right_propeller_2'] = node.step

                    elif ch == 'r':  # roll right
                        node.manual_offsets['left_propeller_2'] = node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == ' ':
                        node.stop_all()
                        print("  All offsets cleared. Vertical thrust: 0.0")

                    elif ch == 'x':
                        exit_clean()

            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        exit_clean()


if __name__ == "__main__":
    main()