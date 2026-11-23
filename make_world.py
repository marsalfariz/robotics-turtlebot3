"""Translate the shared track and robot to Gazebo Classic's SDF plugins."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from line_follower.track import render_world


def add_settings(parent, settings):
    for name, value in settings.items():
        ET.SubElement(parent, name).text = value


def classic_world(template):
    document = ET.fromstring(render_world(template))
    document.set('version', '1.6')
    world = document.find('world')
    for plugin in world.findall('plugin'):
        world.remove(plugin)
    physics = world.find('physics')
    physics.set('type', 'ode')
    ET.SubElement(physics, 'real_time_update_rate').text = '500'

    robot = world.find("model[@name='line_robot']")
    original_drive = robot.find('plugin')
    separation = original_drive.findtext('wheel_separation')
    diameter = str(2 * float(original_drive.findtext('wheel_radius')))
    robot.remove(original_drive)
    drive = ET.SubElement(robot, 'plugin', {
        'name': 'line_robot_diff_drive', 'filename': 'libgazebo_ros_diff_drive.so',
    })
    add_settings(drive, {
        'robotNamespace': '/', 'leftJoint': original_drive.findtext('left_joint'),
        'rightJoint': original_drive.findtext('right_joint'), 'wheelSeparation': separation,
        'wheelDiameter': diameter, 'wheelAcceleration': '0', 'wheelTorque': '1',
        'commandTopic': '/cmd_vel', 'odometryTopic': '/odom', 'odometryFrame': 'odom',
        'robotBaseFrame': 'chassis', 'odometrySource': 'encoder', 'updateRate': '30',
        'publishTf': 'true', 'publishOdomTF': 'true',
    })
    # SDF 1.6 expresses these axes in the model frame using this flag.
    for axis in robot.findall('joint/axis'):
        axis.find('xyz').attrib.pop('expressed_in')
        ET.SubElement(axis, 'use_parent_model_frame').text = 'true'

    camera = robot.find(".//sensor[@name='line_camera']")
    camera.remove(camera.find('topic'))
    bridge = ET.SubElement(camera, 'plugin', {
        'name': 'line_camera_ros', 'filename': 'libgazebo_ros_camera.so',
    })
    add_settings(bridge, {
        'robotNamespace': '/', 'cameraName': 'camera', 'imageTopicName': 'image',
        'cameraInfoTopicName': 'camera_info', 'frameName': 'line_camera',
        'alwaysOn': 'true', 'updateRate': '0',
    })

    gui = ET.SubElement(world, 'gui')
    view = ET.SubElement(gui, 'camera', {'name': 'user_camera'})
    ET.SubElement(view, 'pose').text = f'3 -3.5 7 0 {math.atan2(7, 3.5):.5f} {math.pi / 2:.5f}'
    return ET.ElementTree(document)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('template', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    classic_world(args.template.read_text(encoding='utf-8')).write(
        args.output, encoding='utf-8', xml_declaration=True,
    )


if __name__ == '__main__':
    main()
