import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz = get_package_share_directory('ares_gazebo')
    model_path = os.path.join(gz, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),   # never wait on the internet
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'sim.launch.py')),
            launch_arguments={'world': os.path.join(gz, 'worlds', 'recon_site.world')}.items()),
    ])
