#!/usr/bin/env python3
"""ARES autonomous patrol mission.

Nav2 plans a collision-free path to each waypoint and drives there while SLAM Toolbox
builds the map. For every leg the time, distance driven and result are logged to
~/ares_ws/results/navigation/patrol_<time>.csv, and the finished map is saved next to it.
If the robot gets stuck (frozen or wiggling for 10 s) it frees itself with ares_unstick.py
and tries the same waypoint again (up to 3 times).

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

from ares_unstick import Unsticker

# name, x, y, final heading (deg). Map frame = world frame: the robot starts at (0, 0) facing +x.
WAYPOINTS = [
    ('entrance path',              1.5, -2.4,    0),
    ('lobby',                      4.2, -2.4,    0),
    ('lobby, by inner door',       6.0, -0.6,    0),
    ('office',                     8.5, -0.6,    0),
    ('office, by back-room door', 10.4, -1.0,  -90),
    ('back room (person found)',  10.4, -2.8,  -90),
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
    un = Unsticker(nav)
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
        esc0, result = un.escapes, None
        for attempt in range(4):                      # first try + up to 3 self-rescues
            nav.goToPose(make_pose(nav, x, y, yaw))
            un.reset()
            last_print, stuck = 0.0, False
            while not nav.isTaskComplete():
                el = time.monotonic() - t0
                fb = nav.getFeedback()
                if fb is not None and el - last_print > 3.0:
                    print(f'      {fb.distance_remaining:5.2f} m to go, {el:5.1f} s')
                    last_print = el
                if el > LEG_TIMEOUT:
                    print('      timeout, cancelling this leg')
                    nav.cancelTask()
                    result = 'timeout'
                    break
                if un.stuck():
                    print('      stuck: cancelling and freeing myself')
                    nav.cancelTask()
                    stuck = True
                    break
            while not nav.isTaskComplete():      # wait until the cancel has gone through
                rclpy.spin_once(nav, timeout_sec=0.1)
            if result == 'timeout':
                break
            if stuck:
                if attempt < 3:
                    un.escape()
                    print('      free again, retrying this waypoint')
                    continue
                result = 'failed'
                break
            result = {TaskResult.SUCCEEDED: 'reached', TaskResult.CANCELED: 'timeout',
                      TaskResult.FAILED: 'failed'}.get(nav.getResult(), 'unknown')
            break
        leg_t, leg_d = time.monotonic() - t0, odo['dist'] - d0
        rows.append([i, name, x, y, result, round(leg_t, 1), round(leg_d, 2), un.escapes - esc0])
        print(f'      {result.upper()}: {leg_t:.1f} s, {leg_d:.2f} m driven, '
              f'{un.escapes - esc0} self-rescues\n')

    total_t = time.monotonic() - mission_t0
    path = os.path.join(out_dir, f'patrol_{stamp}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['leg', 'waypoint', 'x', 'y', 'result', 'time_s', 'distance_m', 'self_rescues'])
        w.writerows(rows)
    ok = sum(r[4] == 'reached' for r in rows)
    print('=' * 60)
    print(f'Patrol finished: {ok}/{len(rows)} waypoints reached, '
          f'{sum(r[6] for r in rows):.1f} m driven in {total_t:.0f} s, '
          f'{un.escapes} self-rescues')
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
