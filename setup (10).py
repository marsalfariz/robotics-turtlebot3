import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'line_follower'


def files_in(folder):
    """Return (install_dir, [files]) pairs for every file under folder, keeping subfolders."""
    pairs = []
    for root, _, files in os.walk(folder):
        paths = [os.path.join(root, f) for f in files if not f.startswith('.')]
        if paths:
            pairs.append((os.path.join('share', package_name, root), paths))
    return pairs


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ] + files_in('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rofuy',
    maintainer_email='rofikyd07@gmail.com',
    description='IR line follower robot simulated in Gazebo Fortress',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ir_sensor_node = line_follower.ir_sensor_node:main',
            'line_follower_node = line_follower.line_follower_node:main',
            'lap_logger_node = line_follower.lap_logger_node:main',
        ],
    },
)
