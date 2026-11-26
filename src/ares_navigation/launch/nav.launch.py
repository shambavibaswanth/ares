import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz = get_package_share_directory('ares_gazebo')
    nav = get_package_share_directory('ares_navigation')
    bringup = get_package_share_directory('nav2_bringup')
    params = os.path.join(nav, 'config', 'nav2_params.yaml')

    # 1. Gazebo recon world + robot
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(gz, 'launch', 'recon.launch.py')))

    # 2. Nav2 with live SLAM (slam_toolbox builds the map while Nav2 drives)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={'slam': 'True', 'map': os.path.join(nav, 'maps', 'unused.yaml'), 'use_sim_time': 'True',
                          'params_file': params, 'autostart': 'True'}.items())

    # 3. RViz with the standard Nav2 view (map, costmaps, planned path)
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(bringup, 'rviz', 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    # 4. optional YOLO detector (off by default to keep the laptop fast)
    detector = Node(package='ares_perception', executable='yolo_detector', output='screen',
                    condition=IfCondition(LaunchConfiguration('detect')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('detect', default_value='false'),
        sim,
        TimerAction(period=20.0, actions=[nav2, rviz, detector]),   # let Gazebo spawn first
    ])
