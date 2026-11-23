"""PID line follower for the virtual IR sensor array.

Subscribes:  /ir_sensors   (std_msgs/Float32MultiArray)  0 = black line, 1 = white floor, left -> right
Publishes:   /cmd_vel      (geometry_msgs/Twist)
             /line_error   (std_msgs/Float32)  -1 = line far right, 0 = centred, +1 = line far left

How it works
  1. darkness_i = 1 - value_i                (how much of the line each sensor sees)
  2. Each sensor has a symmetric weight, e.g. 5 sensors -> [+2, +1, 0, -1, -2] (left positive)
  3. line position = sum(darkness_i * weight_i) / sum(darkness_i), scaled to -1..+1  = error
  4. angular.z = kp*error + ki*integral(error) + kd*d(error)/dt   (positive = turn left)
  5. linear.x  = base_speed * (1 - |error|), never below min_speed   (slow down in curves)
  6. Line lost (no sensor dark enough): stop and spin in place toward the side
     where the line was last seen, until any sensor sees it again.

All parameters live in config/follower.yaml.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class LineFollowerNode(Node):

    def __init__(self):
        super().__init__('line_follower_node')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('base_speed', 0.15)      # m/s when the line is centred
        self.declare_parameter('min_speed', 0.05)       # m/s, lowest forward speed while following
        self.declare_parameter('max_angular', 2.0)      # rad/s, limit on turning speed
        self.declare_parameter('search_speed', 1.0)     # rad/s, spin speed when the line is lost
        self.declare_parameter('line_threshold', 0.4)   # a sensor darker than this "sees" the line
        self.declare_parameter('integral_limit', 1.0)   # anti wind-up clamp for the I term
        self.declare_parameter('sensor_timeout', 0.5)   # s without sensor data -> stop

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_pub = self.create_publisher(Float32, '/line_error', 10)
        self.create_subscription(Float32MultiArray, '/ir_sensors', self.on_sensors, 10)
        self.create_timer(0.1, self.watchdog)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.last_side = 1.0        # +1 = line last seen on the left, -1 = on the right
        self.lost = False
        self.last_msg_time = None
        self.get_logger().info('line_follower_node started')

    def p(self, name):
        return self.get_parameter(name).value

    def on_sensors(self, msg):
        values = list(msg.data)
        n = len(values)
        if n == 0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_msg_time = now
        dt = 0.0 if self.prev_time is None else max(now - self.prev_time, 0.0)
        self.prev_time = now

        darkness = [min(max(1.0 - v, 0.0), 1.0) for v in values]
        half = (n - 1) / 2.0
        weights = [half - i for i in range(n)]          # left positive, symmetric
        cmd = Twist()

        if max(darkness) < self.p('line_threshold'):
            # ---- Line lost: spin in place toward where it was last seen ----
            if not self.lost:
                self.get_logger().info(
                    f"Line lost -> searching to the {'left' if self.last_side > 0 else 'right'}")
            self.lost = True
            self.integral = 0.0
            cmd.angular.z = self.last_side * abs(self.p('search_speed'))
            self.err_pub.publish(Float32(data=float(self.last_side)))
            self.cmd_pub.publish(cmd)
            return

        if self.lost:
            self.get_logger().info('Line found again')
            self.lost = False
            self.prev_error = 0.0
            dt = 0.0

        # ---- Line position: weighted average of darkness, scaled to -1..+1 ----
        total = sum(darkness)
        position = sum(d * w for d, w in zip(darkness, weights)) / total
        error = position / half if half > 0 else 0.0
        if abs(error) > 0.05:
            self.last_side = math.copysign(1.0, error)

        # ---- PID ----
        derivative = 0.0
        if dt > 0.0:
            lim = self.p('integral_limit')
            self.integral = min(max(self.integral + error * dt, -lim), lim)
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        angular = self.p('kp') * error + self.p('ki') * self.integral + self.p('kd') * derivative
        max_ang = self.p('max_angular')
        cmd.angular.z = min(max(angular, -max_ang), max_ang)
        cmd.linear.x = max(self.p('min_speed'), self.p('base_speed') * (1.0 - abs(error)))

        self.err_pub.publish(Float32(data=float(error)))
        self.cmd_pub.publish(cmd)

    def watchdog(self):
        """Stop the robot if the sensor data stops arriving."""
        if self.last_msg_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_msg_time > self.p('sensor_timeout'):
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())   # stop the robot on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
