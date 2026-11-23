from setuptools import setup

package = 'line_follower'

setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/launch', ['launch/simulation.launch.py']),
        ('share/' + package + '/worlds', ['worlds/track.sdf.in']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Line Follower Project',
    maintainer_email='maintainer@example.com',
    description='Camera-based line following in Gazebo Fortress.',
    license='UNLICENSED',
    entry_points={
        'console_scripts': [
            'follow = line_follower.controller:main',
            'make_world = line_follower.track:main',
            'verify = line_follower.verify:main',
        ],
    },
)
