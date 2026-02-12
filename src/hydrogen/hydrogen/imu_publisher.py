#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import board
import busio
import adafruit_bno055


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (rad) to quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__("bno055_imu_node")

        # ================= I2C + IMU =================
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c)

        time.sleep(1.0)
        self.get_logger().info("BNO055 initialized")

        # ================= ROS 2 =================
        self.publisher = self.create_publisher(Imu, "/imu", 10)
        self.timer = self.create_timer(0.1, self.publish_imu)  # 10 Hz

        self.frame_id = "imu_link"

    def publish_imu(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ================= ORIENTATION =================
        euler = self.bno.euler  # degrees
        if euler is not None:
            heading, roll, pitch = euler

            # Convert degrees → radians
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(heading)

            qx, qy, qz, qw = euler_to_quaternion(
                roll_rad, pitch_rad, yaw_rad
            )

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
        else:
            msg.orientation.w = 1.0  # identity

        # ================= ANGULAR VELOCITY =================
        gyro = self.bno.gyro  # rad/s
        if gyro is not None:
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

        # ================= LINEAR ACCELERATION =================
        accel = self.bno.linear_acceleration  # m/s^2
        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = BNO055ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
