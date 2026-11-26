#!/usr/bin/env bash
# ARES Phase 4: autonomous navigation (SLAM Toolbox + Nav2) and a patrol mission.
set -e
WS=$HOME/ares_ws
N=$WS/src/ares_navigation
echo "=== ARES autonomous navigation setup ==="
[ "$(which python3)" = "/usr/bin/python3" ] || { echo "ERROR: run ares_env first"; exit 1; }
[ -f $WS/src/ares_gazebo/launch/recon.launch.py ] || { echo "ERROR: recon world missing, run ares_world_setup.sh first"; exit 1; }

# 1. make sure Nav2 + SLAM are installed
MISSING=""
for p in ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox; do
  dpkg -s $p > /dev/null 2>&1 || MISSING="$MISSING $p"
done
if [ -n "$MISSING" ]; then echo "Installing:$MISSING"; sudo apt install -y $MISSING; fi
/usr/bin/python3 -c "import nav2_simple_commander" && echo "[1/5] Nav2 + SLAM Toolbox installed"

# 2. Nav2 parameters: standard file, sized for the ARES robot (0.28 m radius)
mkdir -p $N/config $N/launch $N/maps
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml $N/config/nav2_params.yaml
sed -i 's/robot_radius: 0.22/robot_radius: 0.28/g' $N/config/nav2_params.yaml
echo "[2/5] nav2_params.yaml written ($(grep -c 'robot_radius: 0.28' $N/config/nav2_params.yaml) costmaps set to 0.28 m)"

# 3. one launch file: world + robot + SLAM + Nav2 + RViz
cat > $N/launch/nav.launch.py << 'EOF'
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
EOF
for dep in ares_gazebo nav2_bringup slam_toolbox rviz2; do
  grep -q "<exec_depend>$dep</exec_depend>" $N/package.xml || \
    sed -i "s|<buildtool_depend>ament_cmake</buildtool_depend>|<buildtool_depend>ament_cmake</buildtool_depend>\n  <exec_depend>$dep</exec_depend>|" $N/package.xml
done
grep -q "install(DIRECTORY config launch maps" $N/CMakeLists.txt || \
  sed -i 's|^ament_package()|install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})\n\nament_package()|' $N/CMakeLists.txt
touch $N/maps/.gitkeep
echo "[3/5] nav.launch.py written"

# 4. patrol mission script
cat > $WS/scripts/ares_patrol.py << 'EOF'
#!/usr/bin/env python3
"""ARES autonomous patrol mission.

Nav2 plans a collision-free path to each waypoint and drives there while SLAM Toolbox
builds the map. For every leg the time, distance driven and result are logged to
~/ares_ws/results/navigation/patrol_<time>.csv, and the finished map is saved next to it.

Run (with ares_navigation nav.launch.py already running):
    python3 ~/ares_ws/scripts/ares_patrol.py
"""
import csv
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

# name, x, y, final heading (deg). Map frame = world frame: the robot starts at (0, 0) facing +x.
WAYPOINTS = [
    ('entrance path',              1.5, -2.4,    0),
    ('lobby',                      4.2, -2.4,    0),
    ('lobby, by inner door',       6.0, -0.6,    0),
    ('office',                     8.5, -0.6,    0),
    ('office, by back-room door', 10.4, -1.0,  -90),
    ('back room (person found)',  10.4, -3.3,  -90),
    ('back room centre',           8.5, -3.6,  180),
    ('lobby (return)',             4.2, -2.4,  180),
    ('start (return)',             0.0,  0.0,  180),
]
LEG_TIMEOUT = 180.0   # s per waypoint


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    h = math.radians(yaw_deg) / 2
    p.pose.orientation.z, p.pose.orientation.w = math.sin(h), math.cos(h)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    odo = {'last': None, 'dist': 0.0}

    def on_odom(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if odo['last'] is not None:
            odo['dist'] += math.dist(p, odo['last'])
        odo['last'] = p

    nav.create_subscription(Odometry, 'odom', on_odom, qos_profile_sensor_data)
    print('Waiting for Nav2 to become active ...')
    nav._waitForNodeToActivate('bt_navigator')       # SLAM mode: no AMCL to wait for
    time.sleep(3.0)
    print('Nav2 active. Starting patrol of', len(WAYPOINTS), 'waypoints.\n')

    out_dir = os.path.expanduser('~/ares_ws/results/navigation')
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    rows, mission_t0 = [], time.monotonic()

    for i, (name, x, y, yaw) in enumerate(WAYPOINTS, 1):
        d0, t0 = odo['dist'], time.monotonic()
        print(f'[{i}/{len(WAYPOINTS)}] -> {name} ({x:.1f}, {y:.1f})')
        nav.goToPose(make_pose(nav, x, y, yaw))
        last_print = 0.0
        while not nav.isTaskComplete():
            el = time.monotonic() - t0
            fb = nav.getFeedback()
            if fb is not None and el - last_print > 3.0:
                print(f'      {fb.distance_remaining:5.2f} m to go, {el:5.1f} s')
                last_print = el
            if el > LEG_TIMEOUT:
                print('      timeout, cancelling this leg')
                nav.cancelTask()
        res = nav.getResult()
        result = {TaskResult.SUCCEEDED: 'reached', TaskResult.CANCELED: 'timeout',
                  TaskResult.FAILED: 'failed'}.get(res, 'unknown')
        leg_t, leg_d = time.monotonic() - t0, odo['dist'] - d0
        rows.append([i, name, x, y, result, round(leg_t, 1), round(leg_d, 2)])
        print(f'      {result.upper()}: {leg_t:.1f} s, {leg_d:.2f} m driven\n')

    total_t = time.monotonic() - mission_t0
    path = os.path.join(out_dir, f'patrol_{stamp}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['leg', 'waypoint', 'x', 'y', 'result', 'time_s', 'distance_m'])
        w.writerows(rows)
    ok = sum(r[4] == 'reached' for r in rows)
    print('=' * 60)
    print(f'Patrol finished: {ok}/{len(rows)} waypoints reached, '
          f'{sum(r[6] for r in rows):.1f} m driven in {total_t:.0f} s')
    print(f'Results: {path}')

    map_base = os.path.join(out_dir, f'recon_map_{stamp}')
    try:
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base,
                        '--ros-args', '-p', 'use_sim_time:=true'], timeout=60, check=False)
        print(f'Map saved: {map_base}.pgm / .yaml')
    except Exception as e:
        print('Map save failed:', e)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
chmod +x $WS/scripts/ares_patrol.py
/usr/bin/python3 -m py_compile $WS/scripts/ares_patrol.py && echo "[4/5] scripts/ares_patrol.py written"

# 5. build
cd $WS
colcon build --symlink-install --packages-select ares_navigation
echo "[5/5] Done."
echo "  Terminal 1:  ros2 launch ares_navigation nav.launch.py"
echo "  Terminal 2:  python3 ~/ares_ws/scripts/ares_patrol.py"
