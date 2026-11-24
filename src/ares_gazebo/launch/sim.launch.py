import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_dir = get_package_share_directory('ares_description')
    gz_dir = get_package_share_directory('ares_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(gz_dir, 'worlds', 'obstacle_room.world')
    xacro_file = os.path.join(desc_dir, 'urdf', 'ares.urdf.xacro')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui, 'verbose': 'true'}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ares',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-timeout', '120'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, rsp, spawn,
    ])
