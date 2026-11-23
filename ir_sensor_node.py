"""Virtual IR / photodiode array built from a downward-looking camera.

Gazebo has no IR reflectance sensor, so the robot carries a tiny camera (e.g. 40x4 px)
that looks straight down at the floor. This node samples N small spots across the image,
left to right, like N photodiodes on a real sensor bar.

Subscribes:  /ir_camera/image  (sensor_msgs/Image, rgb8 or mono8)
Publishes:   /ir_sensors       (std_msgs/Float32MultiArray)
             data[0] = leftmost sensor ... data[N-1] = rightmost sensor
             0.0 = dark (black line), 1.0 = bright (white floor)

Parameters:
  num_sensors      int    5        number of virtual sensors
  sensor_spacing   float  0.015    distance between neighbouring sensors [m]
  camera_height    float  0.04     camera height above the floor [m]  (must match model.sdf)
  camera_hfov      float  1.5708   camera horizontal field of view [rad] (must match model.sdf)
  spot_size        int    3        width of each sensor spot [px], averaged like a photodiode
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class IrSensorNode(Node):

    def __init__(self):
        super().__init__('ir_sensor_node')
        self.declare_parameter('num_sensors', 5)
        self.declare_parameter('sensor_spacing', 0.015)
        self.declare_parameter('camera_height', 0.04)
        self.declare_parameter('camera_hfov', 1.5708)
        self.declare_parameter('spot_size', 3)

        self.pub = self.create_publisher(Float32MultiArray, '/ir_sensors', 10)
        self.sub = self.create_subscription(Image, '/ir_camera/image', self.on_image, 10)
        self.columns = None  # pixel column of each sensor, computed on the first image
        self.get_logger().info('ir_sensor_node started, waiting for /ir_camera/image ...')

    def compute_columns(self, width):
        n = self.get_parameter('num_sensors').value
        spacing = self.get_parameter('sensor_spacing').value
        height = self.get_parameter('camera_height').value
        hfov = self.get_parameter('camera_hfov').value

        # Width of the floor strip the camera sees, and metres per pixel
        floor_width = 2.0 * height * math.tan(hfov / 2.0)
        m_per_px = floor_width / width

        cols = []
        for i in range(n):
            # Lateral offset of sensor i from the centre; i = 0 is the leftmost (+y side)
            offset = ((n - 1) / 2.0 - i) * spacing
            # Image column 0 is the robot's left, so a positive (left) offset means a smaller column
            col = (width - 1) / 2.0 - offset / m_per_px
            cols.append(int(round(min(max(col, 0), width - 1))))

        if (n - 1) * spacing > floor_width:
            self.get_logger().warn(
                f'Sensor span {(n - 1) * spacing:.3f} m is wider than the camera view '
                f'{floor_width:.3f} m; outer sensors are clamped to the image edge.')
        self.get_logger().info(
            f'{n} sensors, spacing {spacing * 100:.1f} cm, camera sees {floor_width * 100:.1f} cm '
            f'-> pixel columns {cols} of {width}')
        return cols

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            self.get_logger().error(f'Unsupported image encoding {msg.encoding}', once=True)
            return
        channels = 1 if msg.encoding == 'mono8' else 3
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        gray = img.mean(axis=2)  # brightness 0..255 per pixel

        if self.columns is None:
            self.columns = self.compute_columns(msg.width)

        half = self.get_parameter('spot_size').value // 2
        values = []
        for c in self.columns:
            spot = gray[:, max(c - half, 0):c + half + 1]
            values.append(float(spot.mean()) / 255.0)

        out = Float32MultiArray()
        out.data = values
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IrSensorNode()
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
