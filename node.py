#!/usr/bin/env python3
"""ROS 1 transport for the shared camera controller and verifier."""

import argparse
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from line_follower.verification import OdometrySample, Verification
from line_follower.vision import ENCODING_CHANNELS, line_offset, velocity


def follow(shutdown):
    speed = rospy.get_param('~speed_mps', 0.19)
    gain = rospy.get_param('~proportional_gain', 3.1)
    max_yaw = rospy.get_param('~max_yaw_rad_s', 1.25)
    for value in (speed, gain, max_yaw):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Controller parameters must be finite and positive')
    command = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    observation = rospy.Publisher('/line_offset', Float64, queue_size=10)
    state = (None, -math.inf)

    def on_frame(msg):
        nonlocal state
        if msg.encoding not in ENCODING_CHANNELS:
            rospy.logwarn_throttle(5, 'Unsupported camera encoding: %s', msg.encoding)
        position = line_offset(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        state = position, time.monotonic()
        observation.publish(Float64(data=position if position is not None else math.nan))

    camera = rospy.Subscriber('/camera/image', Image, on_frame, queue_size=1)
    rospy.on_shutdown(lambda: command.publish(Twist()))
    try:
        while not shutdown.is_set():
            offset, received = state
            linear, angular = velocity(
                offset, time.monotonic() - received, speed=speed, gain=gain, max_yaw=max_yaw,
            )
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            command.publish(msg)
            shutdown.wait(0.05)
    finally:
        camera.unregister()


def verify(shutdown):
    verification = Verification(rospy.get_param('~evidence_dir'), rospy.loginfo)
    lock = threading.Lock()

    def on_odom(msg):
        pos, twist = msg.pose.pose.position, msg.twist.twist
        with lock:
            verification.on_odom(OdometrySample(
                msg.header.stamp.to_sec(), pos.x, pos.y, twist.linear.x, twist.angular.z,
            ))

    def on_command(msg):
        with lock:
            verification.on_command((msg.linear.x, msg.angular.z))

    def on_offset(msg):
        with lock:
            verification.on_offset(msg.data)

    subscribers = [
        rospy.Subscriber('/odom', Odometry, on_odom, queue_size=10),
        rospy.Subscriber('/cmd_vel', Twist, on_command, queue_size=10),
        rospy.Subscriber('/line_offset', Float64, on_offset, queue_size=10),
    ]
    try:
        while not shutdown.is_set() and verification.result is None:
            with lock:
                verification.sample()
            shutdown.wait(0.10)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        verification.csv_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('follow', 'verify'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('line_follower' if args.mode == 'follow' else 'line_follower_verifier')
    shutdown = threading.Event()
    rospy.on_shutdown(shutdown.set)
    if args.mode == 'follow':
        follow(shutdown)
    else:
        verify(shutdown)


if __name__ == '__main__':
    main()
