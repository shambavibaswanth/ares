from glob import glob
from setuptools import find_packages, setup

package_name = 'ares_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shambavi H B',
    maintainer_email='sshambavi387@gmail.com',
    description='ARES obstacle avoidance: safety guard and reactive explore mode',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'obstacle_avoidance = ares_autonomy.obstacle_avoidance:main']},
)
