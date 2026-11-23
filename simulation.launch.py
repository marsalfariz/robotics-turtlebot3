from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')
    verify = LaunchConfiguration('verify')
    evidence = LaunchConfiguration('evidence_dir')

    gui_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        condition=UnlessCondition(headless),
        output='screen',
    )
    server_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', world],
        condition=IfCondition(headless),
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        output='screen',
    )
    controller = Node(package='line_follower', executable='follow', output='screen')
    verifier = Node(
        package='line_follower',
        executable='verify',
        parameters=[{'evidence_dir': ParameterValue(evidence, value_type=str)}],
        condition=IfCondition(verify),
        output='screen',
    )
    run_processes = [gui_sim, server_sim, controller, bridge, verifier]
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Line follower process exited'))],
        ))
        for process in run_processes
    ]
    return LaunchDescription([
        DeclareLaunchArgument('world_file'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('verify', default_value='false'),
        DeclareLaunchArgument('evidence_dir', default_value='.'),
        *shutdown_handlers,
        *run_processes,
    ])
