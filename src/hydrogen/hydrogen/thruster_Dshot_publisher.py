#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import signal
import sys

# ============================================================
# UART CONFIG
# ============================================================
UART_PORT = "/dev/serial0"     # use /dev/ttyAMA0 if needed
UART_BAUD = 115200

# ============================================================
# THRUST / DIRECTION CONFIG
# ============================================================
MAX_THRUST = 100.0

CW_MIN = 0
CW_MAX = 1000
CCW_MIN = 1001
CCW_MAX = 2000

# ============================================================
# LOGGING CONFIG
# ============================================================
LOG_UART = True          # enable / disable logging
LOG_RATE_DIV = 50        # log every N cycles (500Hz / 50 = 10Hz)


# ============================================================
# THRUSTER INDEX MAP (DO NOT CHANGE)
# ============================================================
# 0 -> back
# 1 -> right_propeller_2
# 2 -> left_propeller_2
# 3 -> right_propeller_1
# 4 -> left_propeller_1
THRUSTER_INDEX = {
    "back_propeller": 0,
    "right_propeller_2": 1,
    "left_propeller_2": 2,
    "right_propeller_1": 3,
    "left_propeller_1": 4,
}

TOPICS = {
    "back_propeller": "/hydrogen/back_propeller/cmd_thrust",
    "right_propeller_1": "/hydrogen/right_propeller_1/cmd_thrust",
    "right_propeller_2": "/hydrogen/right_propeller_2/cmd_thrust",
    "left_propeller_1": "/hydrogen/left_propeller_1/cmd_thrust",
    "left_propeller_2": "/hydrogen/left_propeller_2/cmd_thrust",
}

# ============================================================
# MAPPING FUNCTIONS
# ============================================================
def thrust_to_throttle(thrust):
    thrust = max(-MAX_THRUST, min(MAX_THRUST, thrust))
    mag = abs(thrust) / MAX_THRUST

    if thrust >= 0:
        return int(CW_MIN + mag * (CW_MAX - CW_MIN))
    else:
        return int(CCW_MIN + mag * (CCW_MAX - CCW_MIN))


def make_dshot_frame(throttle):
    throttle &= 0x7FF                     # 11-bit
    packet = (throttle << 1) | 0          # telemetry bit = 0
    crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
    frame = (packet << 4) | crc

    return (frame >> 8) & 0xFF, frame & 0xFF


# ============================================================
# ROS NODE
# ============================================================
class ThrusterUARTBridge(Node):
    def __init__(self):
        super().__init__("thruster_uart_bridge")

        self.thrust = [0.0] * 5

        # UART
        self.ser = serial.Serial(
            UART_PORT,
            UART_BAUD,
            timeout=0
        )

        # Subscriptions
        for name, topic in TOPICS.items():
            self.create_subscription(
                Float64,
                topic,
                lambda msg, n=name: self.cb(msg, n),
                10
            )

        # 500 Hz update
        self.timer = self.create_timer(0.002, self.send_uart)
        self._log_counter = 0


        self.get_logger().info("Thruster UART bridge running")

    def cb(self, msg, name):
        idx = THRUSTER_INDEX[name]
        self.thrust[idx] = msg.data

    def send_uart(self):
        self._log_counter += 1
        do_log = LOG_UART and (self._log_counter % LOG_RATE_DIV == 0)

        for i in range(5):
            thrust = self.thrust[i]
            throttle = thrust_to_throttle(thrust)
            hi, lo = make_dshot_frame(throttle)

            packet = bytes([
                0x10 | i,
                hi,
                lo
            ])

            self.ser.write(packet)

            if do_log:
                self.get_logger().info(
                    f"[THRUSTER {i}] "
                    f"thrust={thrust:+7.2f} "
                    f"throttle={throttle:4d} "
                    f"bytes=[0x{packet[0]:02X} 0x{packet[1]:02X} 0x{packet[2]:02X}]"
                )


    def stop_all(self):
        for i in range(5):
            hi, lo = make_dshot_frame(0)
            self.ser.write(bytes([0x10 | i, hi, lo]))


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()
    node = ThrusterUARTBridge()

    def shutdown(*_):
        node.get_logger().info("Stopping all thrusters")
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
