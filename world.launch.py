"""Open the line track world in Gazebo Fortress.

    ros2 launch line_follower world.launch.py
    ros2 launch line_follower world.launch.py headless:=true   # no GUI window
    ros2 launch line_follower world.launch.py software_gl:=false  # PC with a real GPU (faster)
    ros2 launch line_follower world.launch.py gui_config:=<path>  # custom GUI layout (video recording)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    render_engine = LaunchConfiguration('render_engine')
    software_gl = LaunchConfiguration('software_gl')
    gui_config = LaunchConfiguration('gui_config')

    # Let Gazebo find our models/ folder (used from the next step on)
    resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_share, 'models') + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))

    # -r = start the simulation running immediately; -s = server only (no GUI)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': PythonExpression([
            "'-r --render-engine ", render_engine, " ' + ",
            "('--gui-config ", gui_config, " ' if '", gui_config, "' else '') + '", world, "'"])}.items(),
        condition=UnlessCondition(headless))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': PythonExpression(["'-r -s --render-engine ", render_engine, " ' + '", world, "'"])}.items(),
        condition=IfCondition(headless))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'line_track.sdf'),
                              description='Full path to the world SDF file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = run Gazebo without the GUI window'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
                              description='Gazebo render engine: ogre2 (default) or ogre'),
        # Virtual machines (VirtualBox/Parallels/VMware) often have a GPU driver that crashes
        # Gazebo. Forcing CPU (software) OpenGL is slower but works everywhere.
        DeclareLaunchArgument('gui_config', default_value='',
                              description='Optional Gazebo GUI layout file (e.g. config/gui_video.config)'),
        DeclareLaunchArgument('software_gl', default_value='true',
                              description='true = render with the CPU (needed in most VMs)'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(software_gl)),
        resource_path,
        gazebo_gui,
        gazebo_headless,
    ])
