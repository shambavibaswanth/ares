from setuptools import find_packages, setup

package_name = 'ares_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shambavi H B',
    maintainer_email='sshambavi387@gmail.com',
    description='ARES perception: YOLOv8 object detection with LiDAR distance fusion',
    license='Apache-2.0',
    entry_points={'console_scripts': ['yolo_detector = ares_perception.yolo_detector:main']},
)
