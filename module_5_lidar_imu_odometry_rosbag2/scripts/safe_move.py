#!/usr/bin/env python3

import sys
import time
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


STOP_REQUESTED = False


def signal_handler(signum, frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


class SafeMove(Node):

    def __init__(self):
        super().__init__('m05_safe_move')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def send(self, speed):

        msg = Twist()

        msg.linear.x = float(speed)
        msg.angular.z = 0.0

        self.pub.publish(msg)

    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Kirim STOP berulang selama ~1.5 detik
        end_time = time.monotonic() + 1.5

        while time.monotonic() < end_time:

            self.pub.publish(msg)

            rclpy.spin_once(
                self,
                timeout_sec=0.0
            )

            time.sleep(0.025)


def main():

    if len(sys.argv) != 3:

        print("Usage: safe_move.py SPEED DURATION")
        sys.exit(1)

    speed = float(sys.argv[1])
    duration = float(sys.argv[2])

    rclpy.init()

    node = SafeMove()

    start = time.monotonic()

    try:

        print("")
        print("======================================")
        print(" SAFE MOTION")
        print("======================================")
        print(f"Speed     : {speed:.3f} m/s")
        print(f"Duration  : {duration:.2f} s")
        print("AUTOSTOP  : ON")
        print("======================================")
        print("")

        # Pastikan robot diam sebelum mulai
        node.stop_robot()

        start = time.monotonic()

        while not STOP_REQUESTED:

            elapsed = time.monotonic() - start

            if elapsed >= duration:
                break

            node.send(speed)

            remaining = duration - elapsed

            print(
                f"\rMOVING: {elapsed:5.2f}s / {duration:.2f}s "
                f"| remaining {remaining:5.2f}s",
                end="",
                flush=True
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.0
            )

            time.sleep(0.05)

    finally:

        print("")
        print("")
        print("AUTOSTOP -> CMD_VEL = 0")

        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
