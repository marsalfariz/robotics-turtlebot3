#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


rclpy.init()

node = Node('m05_emergency_stop')

pub = node.create_publisher(
    Twist,
    '/cmd_vel',
    10
)

msg = Twist()

msg.linear.x = 0.0
msg.angular.z = 0.0

end_time = time.monotonic() + 2.0

while time.monotonic() < end_time:

    pub.publish(msg)

    rclpy.spin_once(
        node,
        timeout_sec=0.0
    )

    time.sleep(0.02)

node.destroy_node()

rclpy.shutdown()
