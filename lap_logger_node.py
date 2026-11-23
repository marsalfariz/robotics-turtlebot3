"""Log the run to CSV and measure lap time.

Subscribes:  /odom (nav_msgs/Odometry)          wheel odometry, starts at (0, 0) at the spawn pose
             /ground_truth (nav_msgs/Odometry)  true pose in world coordinates (from Gazebo)
             /cmd_vel (geometry_msgs/Twist), /line_error (std_msgs/Float32)
Writes:      <log_dir>/run_YYYYmmdd_HHMMSS.csv      columns: time, error, linear_x, angular_z, x, y, gt_x, gt_y
                 x, y       = /odom position;  gt_x, gt_y = /ground_truth position (empty if not available)
             <log_dir>/run_YYYYmmdd_HHMMSS_laps.csv columns: lap, start_time, end_time, lap_time, distance, source
             (time = simulation time in seconds since the robot first moved)

Lap detection: a lap is finished when the robot comes back within `lap_radius` of its start point
after driving at least `min_lap_distance`. It uses /ground_truth when available (wheel odometry
drifts when the wheels slip at high speed) and falls back to /odom otherwise.
"""
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class LapLoggerNode(Node):

    def __init__(self):
        super().__init__('lap_logger_node')
        self.declare_parameter('log_dir', '~/line_follower_ws/logs')
        self.declare_parameter('log_rate', 10.0)          # CSV rows per second (sim time)
        self.declare_parameter('lap_radius', 0.15)        # m
        self.declare_parameter('min_lap_distance', 5.0)   # m
        # true: wait for /ground_truth before starting the lap timer, and use only it for laps
        self.declare_parameter('use_ground_truth', True)

        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'run_{stamp}.csv')
        self.lap_path = os.path.join(log_dir, f'run_{stamp}_laps.csv')
        self.csv = open(self.csv_path, 'w', buffering=1)
        self.csv.write('time,error,linear_x,angular_z,x,y,gt_x,gt_y\n')
        self.lap_csv = open(self.lap_path, 'w', buffering=1)
        self.lap_csv.write('lap,start_time,end_time,lap_time,distance,source\n')

        self.error = 0.0
        self.cmd = Twist()
        self.t0 = None              # sim time when the robot first moved
        self.last_row_t = -1e9
        self.prev_xy = None
        self.lap_start_xy = None
        self.lap_start_t = None
        self.lap_distance = 0.0
        self.lap = 0
        self.gt_xy = None           # latest /ground_truth position
        self.source = None          # 'ground_truth' or 'odom', fixed when the lap timer starts

        self.create_subscription(Float32, '/line_error', self.on_error, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_gt, 10)
        self.get_logger().info(f'Logging to {self.csv_path}')

    def on_error(self, msg):
        self.error = msg.data

    def on_cmd(self, msg):
        self.cmd = msg

    def on_gt(self, msg):
        self.gt_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        if self.t0 is None:
            if abs(msg.twist.twist.linear.x) < 1e-3 and abs(msg.twist.twist.angular.z) < 1e-3:
                return                                  # wait until the robot starts moving
            if self.get_parameter('use_ground_truth').value and self.gt_xy is None:
                return                                  # wait for the first /ground_truth message
            # Fix the position source for the whole run: never mix odom and world coordinates
            self.source = 'ground_truth' if self.gt_xy is not None else 'odom'

        lx, ly = self.gt_xy if self.source == 'ground_truth' else (x, y)
        source = self.source

        if self.t0 is None:
            self.t0 = now
            self.lap_start_t = now
            self.lap_start_xy = (lx, ly)
            self.prev_xy = (lx, ly)
            self.get_logger().info(f'Robot started moving, lap timer running (lap source: {source})')

        self.lap_distance += math.hypot(lx - self.prev_xy[0], ly - self.prev_xy[1])
        self.prev_xy = (lx, ly)
        t = now - self.t0

        if now - self.last_row_t >= 1.0 / self.get_parameter('log_rate').value:
            self.last_row_t = now
            gt = f'{self.gt_xy[0]:.4f},{self.gt_xy[1]:.4f}' if self.gt_xy else ','
            self.csv.write(f'{t:.3f},{self.error:.4f},{self.cmd.linear.x:.4f},'
                           f'{self.cmd.angular.z:.4f},{x:.4f},{y:.4f},{gt}\n')

        dist_to_start = math.hypot(lx - self.lap_start_xy[0], ly - self.lap_start_xy[1])
        if (self.lap_distance >= self.get_parameter('min_lap_distance').value
                and dist_to_start <= self.get_parameter('lap_radius').value):
            self.lap += 1
            lap_time = now - self.lap_start_t
            self.lap_csv.write(f'{self.lap},{self.lap_start_t - self.t0:.3f},{t:.3f},'
                               f'{lap_time:.3f},{self.lap_distance:.3f},{source}\n')
            self.get_logger().info(
                f'LAP {self.lap} finished: {lap_time:.2f} s (sim time), {self.lap_distance:.2f} m ({source})')
            self.lap_start_t = now
            self.lap_distance = 0.0

    def destroy_node(self):
        self.csv.close()
        self.lap_csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LapLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
