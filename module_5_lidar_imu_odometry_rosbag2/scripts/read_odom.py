import sys
import math

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions
)

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


bag = sys.argv[1]

reader = SequentialReader()

reader.open(
    StorageOptions(
        uri=bag,
        storage_id='sqlite3'
    ),
    ConverterOptions('', '')
)

xs = []
ys = []

while reader.has_next():

    topic, data, timestamp = reader.read_next()

    if topic == '/odom':

        msg = deserialize_message(
            data,
            Odometry
        )

        xs.append(
            msg.pose.pose.position.x
        )

        ys.append(
            msg.pose.pose.position.y
        )


if len(xs) < 2:

    print("ERROR")
    sys.exit(1)


x_start = xs[0]
y_start = ys[0]

x_end = xs[-1]
y_end = ys[-1]

dx = x_end - x_start
dy = y_end - y_start

distance = math.hypot(
    dx,
    dy
)

e_odom = math.hypot(
    dx - 1.0,
    dy
)

print(
    f"{x_start:.6f},"
    f"{y_start:.6f},"
    f"{x_end:.6f},"
    f"{y_end:.6f},"
    f"{dx:.6f},"
    f"{dy:.6f},"
    f"{distance:.6f},"
    f"{e_odom:.6f}"
)
