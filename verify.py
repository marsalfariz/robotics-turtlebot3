"""Run the straight, bend and line-loss checks using live ROS topics."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from .verification import OdometrySample, Verification


class Verifier(Node):
    def __init__(self):
        super().__init__('line_follower_verifier')
        directory = self.declare_parameter('evidence_dir', '.').value
        self.verification = Verification(directory, self.get_logger().info)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(Float64, '/line_offset', self.on_offset, 10)
        self.create_timer(0.10, self.verification.sample)

    def on_odom(self, msg):
        position = msg.pose.pose.position
        twist = msg.twist.twist
        stamp = msg.header.stamp
        self.verification.on_odom(OdometrySample(
            stamp.sec + stamp.nanosec / 1e9, position.x, position.y,
            twist.linear.x, twist.angular.z,
        ))

    def on_command(self, msg):
        self.verification.on_command((msg.linear.x, msg.angular.z))

    def on_offset(self, msg):
        self.verification.on_offset(msg.data)


def main():
    rclpy.init()
    node = None
    try:
        node = Verifier()
        while rclpy.ok() and node.verification.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.verification.csv_file.close()
            node.destroy_node()
        rclpy.try_shutdown()
