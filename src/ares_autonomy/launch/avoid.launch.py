import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sim(context):
    gz = get_package_share_directory('ares_gazebo')
    world = LaunchConfiguration('world').perform(context)
    f = os.path.join(gz, 'launch', 'recon.launch.py' if world == 'recon' else 'sim.launch.py')
    if not os.path.exists(f):
        f = os.path.join(gz, 'launch', 'sim.launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(f))]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ares_autonomy'), 'config', 'avoidance.yaml')
    avoid = Node(package='ares_autonomy', executable='obstacle_avoidance', output='screen',
                 parameters=[cfg, {'mode': LaunchConfiguration('mode')}])
    detector = Node(package='ares_perception', executable='yolo_detector', output='screen',
                    condition=IfCondition(LaunchConfiguration('detect')))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='recon', description='recon or room'),
        DeclareLaunchArgument('mode', default_value='guard', description='guard, auto or off'),
        DeclareLaunchArgument('detect', default_value='true', description='run YOLO detector'),
        OpaqueFunction(function=_sim),
        avoid,
        detector,
    ])
