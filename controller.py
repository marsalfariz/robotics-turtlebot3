import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from .vision import ENCODING_CHANNELS, line_offset, velocity


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.speed = self.declare_parameter('speed_mps', 0.19).value
        self.gain = self.declare_parameter('proportional_gain', 3.1).value
        self.max_yaw = self.declare_parameter('max_yaw_rad_s', 1.25).value
        for value in (self.speed, self.gain, self.max_yaw):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Controller parameters must be finite and positive')
        self.offset = None
        self.last_frame = -math.inf
        self.unsupported_encoding = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.observation = self.create_publisher(Float64, '/line_offset', 10)
        self.create_subscription(Image, '/camera/image', self.on_frame, qos_profile_sensor_data)
        self.create_timer(0.05, self.publish_velocity)

    def on_frame(self, msg):
        if msg.encoding not in ENCODING_CHANNELS and msg.encoding != self.unsupported_encoding:
            self.get_logger().warning(f'Unsupported camera encoding: {msg.encoding}')
            self.unsupported_encoding = msg.encoding
        self.offset = line_offset(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        self.last_frame = time.monotonic()
        self.observation.publish(Float64(data=self.offset if self.offset is not None else math.nan))

    def publish_velocity(self):
        linear, angular = velocity(
            self.offset, time.monotonic() - self.last_frame,
            speed=self.speed, gain=self.gain, max_yaw=self.max_yaw,
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd.publish(command)


def main():
    rclpy.init()
    node = None
    try:
        node = LineFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.cmd.publish(Twist())
            node.destroy_node()
        rclpy.try_shutdown()
