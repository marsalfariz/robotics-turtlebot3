"""Full simulation: world + line_robot + ROS<->Gazebo bridge + virtual IR sensor node.

    ros2 launch line_follower sim.launch.py
    ros2 launch line_follower sim.launch.py headless:=true      # no Gazebo window
    ros2 launch line_follower sim.launch.py software_gl:=false  # PC with a real GPU
    ros2 launch line_follower sim.launch.py follow:=true        # robot follows the line + CSV log

Topics on the ROS side:
    /cmd_vel           geometry_msgs/Twist          ROS -> Gazebo (drive the robot)
    /odom              nav_msgs/Odometry            Gazebo -> ROS (wheel odometry, drifts when wheels slip)
    /ground_truth      nav_msgs/Odometry            Gazebo -> ROS (true pose, for evaluation only)
    /ir_camera/image   sensor_msgs/Image            Gazebo -> ROS (40x4 px floor camera)
    /ir_sensors        std_msgs/Float32MultiArray   from ir_sensor_node (0 = black, 1 = white)
    /clock             rosgraph_msgs/Clock          simulation time
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    robot_sdf = os.path.join(pkg_share, 'models', 'line_robot', 'model.sdf')

    # 1. Gazebo + world (reuses world.launch.py, passes its arguments through)
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'world.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'software_gl': LaunchConfiguration('software_gl'),
            'gui_config': LaunchConfiguration('gui_config'),
        }.items())

    # 2. Spawn the robot on the bottom straight of the track, facing +x
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'line_track', '-file', robot_sdf, '-name', 'line_robot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw')])

    # 3. Bridge: '@ROS type ] gz type' = ROS->Gazebo, '[' = Gazebo->ROS
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ir_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ])

    # 4. Virtual IR sensor array
    ir_node = Node(
        package='line_follower', executable='ir_sensor_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_sensors': LaunchConfiguration('num_sensors'),
            'sensor_spacing': LaunchConfiguration('sensor_spacing'),
        }])

    # 5. Controller + CSV/lap logger, only with follow:=true
    params_file = LaunchConfiguration('params_file')
    follower = Node(
        package='line_follower', executable='line_follower_node', output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('follow')))
    logger = Node(
        package='line_follower', executable='lap_logger_node', output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('follow')))

    return LaunchDescription([
        DeclareLaunchArgument('follow', default_value='false',
                              description='true = start line_follower_node and lap_logger_node'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_share, 'config', 'follower.yaml'),
                              description='YAML file with the controller/logger parameters'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('software_gl', default_value='true'),
        DeclareLaunchArgument('gui_config', default_value='',
                              description='Optional Gazebo GUI layout, e.g. config/gui_video.config'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-1.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('num_sensors', default_value='5'),
        DeclareLaunchArgument('sensor_spacing', default_value='0.015',
                              description='Distance between IR sensors [m]'),
        world,
        spawn,
        bridge,
        ir_node,
        follower,
        logger,
    ])
