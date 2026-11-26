#!/usr/bin/env bash
# ARES: frontier exploration (robot goes everywhere it can reach) + slimmer Nav2 wall padding.
set -e
WS=$HOME/ares_ws
N=$WS/src/ares_navigation
echo "=== ARES exploration setup ==="
[ "$(which python3)" = "/usr/bin/python3" ] || { echo "ERROR: run ares_env first"; exit 1; }
[ -f $N/config/nav2_params.yaml ] || { echo "ERROR: run ares_nav_setup.sh first"; exit 1; }

# 1. slimmer padding around walls so doorways and the partition gap stay open
sed -i 's/inflation_radius: 0.55/inflation_radius: 0.40/g; s/cost_scaling_factor: 3.0/cost_scaling_factor: 4.0/g' $N/config/nav2_params.yaml
echo "[1/3] Nav2 padding: $(grep -c 'inflation_radius: 0.40' $N/config/nav2_params.yaml) costmaps at 0.40 m"

# 2. frontier explorer
cat > $WS/scripts/ares_explore.py << 'EOF'
#!/usr/bin/env python3
"""ARES autonomous frontier exploration (SLAM Toolbox + Nav2).

The robot explores everywhere it can reach, with no waypoints given:
  1. read the live SLAM map (/map)
  2. find "frontiers": free cells next to unknown space, grouped into clusters
  3. drop frontiers too close to walls or ones that already failed
  4. send Nav2 to the best one (near, large); re-plan when that area becomes known
  5. when no frontiers remain: drive home, save the map, write results

Results: ~/ares_ws/results/navigation/explore_<time>.csv (area explored over time)
         ~/ares_ws/results/navigation/explore_map_<time>.pgm/.yaml

Run (with ares_navigation nav.launch.py already running):
    python3 ~/ares_ws/scripts/ares_explore.py
"""
import csv
import math
import os
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

MIN_CLUSTER = 8          # cells (0.05 m each): ignore tiny frontier specks
WALL_CLEAR = 0.35        # m: frontier goal must be this far from any obstacle
GOAL_TIMEOUT = 60.0      # s: give up on a frontier after this long
BLACKLIST_R = 0.6        # m: failed goals block frontiers this close
RETRY_AFTER = 90.0       # s: a failed frontier may be tried again after this (map has grown)
SIZE_BONUS = 0.02        # m of distance saved per frontier cell (prefer big openings)
MAX_RUNTIME = 30 * 60    # s: stop exploring after 30 min no matter what


# ---------- pure helpers (no ROS, unit-testable) ----------
def frontier_mask(grid):
    """grid: 2-D int array (-1 unknown, 0..100 occupancy). Free cells touching unknown."""
    free = (grid >= 0) & (grid < 25)
    unk = grid == -1
    nb = np.zeros_like(unk)
    nb[1:, :] |= unk[:-1, :]
    nb[:-1, :] |= unk[1:, :]
    nb[:, 1:] |= unk[:, :-1]
    nb[:, :-1] |= unk[:, 1:]
    return free & nb


def dilate(mask, r):
    """Square dilation by r cells (no scipy needed)."""
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy == 0 and dx == 0:
                continue
            ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
            xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
            out[yd, xd] |= mask[ys, xs]
    return out


def clusters(mask):
    """8-connected groups of True cells -> list of (N,2) arrays of (row, col)."""
    seen = np.zeros_like(mask)
    out = []
    h, w = mask.shape
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack, cells = [(r0, c0)], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        out.append(np.array(cells))
    return out


def pick_frontier(grid, res, ox, oy, robot_xy, blacklist):
    """Best frontier goal in world coords, or None. Returns (x, y, size, n_frontiers)."""
    fr = frontier_mask(grid)
    occ = grid >= 65
    fr &= ~dilate(occ, max(1, int(round(WALL_CLEAR / res))))
    best, n = None, 0
    for cl in clusters(fr):
        if len(cl) < MIN_CLUSTER:
            continue
        n += 1
        cy, cx = cl.mean(axis=0)
        i = int(np.argmin((cl[:, 0] - cy) ** 2 + (cl[:, 1] - cx) ** 2))   # cell nearest centroid
        gx, gy = ox + (cl[i, 1] + 0.5) * res, oy + (cl[i, 0] + 0.5) * res
        if any(math.hypot(gx - bx, gy - by) < BLACKLIST_R for bx, by in blacklist):
            continue
        d = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if d < 0.4:                       # we are basically there already
            continue
        score = d - SIZE_BONUS * len(cl)
        if best is None or score < best[0]:
            best = (score, gx, gy, len(cl))
    return (None if best is None else (best[1], best[2], best[3], n)), n


class Explorer:
    def __init__(self):
        self.nav = BasicNavigator()
        self.map = None
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.nav.create_subscription(OccupancyGrid, 'map', self.on_map, q)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self.nav)

    def on_map(self, m):
        self.map = m

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time(),
                                         timeout=Duration(seconds=0.5))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def grid(self):
        m = self.map
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        return g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y

    def explored_m2(self):
        g, res, _, _ = self.grid()
        return float(((g >= 0) & (g < 25)).sum()) * res * res

    def goal(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.w = 1.0
        return p


def main():
    rclpy.init()
    ex = Explorer()
    print('Waiting for Nav2 ...')
    ex.nav._waitForNodeToActivate('bt_navigator')
    while ex.map is None or ex.robot_xy() is None:
        print('  waiting for /map and robot position ...')
        ex.spin(1.0)
    home = ex.robot_xy()
    print(f'Exploring from home {home[0]:.1f}, {home[1]:.1f}\n')

    out = os.path.expanduser('~/ares_ws/results/navigation')
    os.makedirs(out, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    log = [('time_s', 'explored_m2', 'goals_reached', 'goals_failed', 'frontiers_left')]
    fails, reached, failed, empty_checks = {}, 0, 0, 0   # (x, y) -> (count, last_fail_time)
    streak, last_good = [], home     # consecutive failures, last place reached successfully
    t0 = time.monotonic()

    while time.monotonic() - t0 < MAX_RUNTIME:
        ex.spin(0.5)
        g, res, ox, oy = ex.grid()
        pos = ex.robot_xy() or home
        now = time.monotonic()
        blacklist = [k for k, (cnt, tf) in fails.items() if cnt >= 2 or now - tf < RETRY_AFTER]
        choice, n = pick_frontier(g, res, ox, oy, pos, blacklist)
        log.append((round(time.monotonic() - t0, 1), round(ex.explored_m2(), 1), reached, failed, n))
        if choice is None:
            empty_checks += 1
            waiting = [k for k, (cnt, tf) in fails.items() if cnt < 2]
            print(f'  no reachable frontier right now ({empty_checks}/3), '
                  f'{len(waiting)} failed frontier(s) can be retried later')
            if empty_checks >= 3 and not waiting:
                break
            if empty_checks >= 3:                    # retry the failed-once ones now
                for k in waiting:
                    fails[k] = (fails[k][0], 0.0)
                empty_checks = 0
            ex.spin(3.0)
            continue
        empty_checks = 0
        gx, gy, size, _ = choice
        print(f'[{time.monotonic() - t0:5.0f} s] {n} frontiers, going to ({gx:.1f}, {gy:.1f}) '
              f'size {size}, explored {ex.explored_m2():.0f} m2')
        ex.nav.goToPose(ex.goal(gx, gy))
        t_goal, result = time.monotonic(), None
        while not ex.nav.isTaskComplete():
            el = time.monotonic() - t_goal
            if el > GOAL_TIMEOUT:
                ex.nav.cancelTask()
                result = 'timeout'
                break
            if el > 4.0 and int(el) % 4 == 0:     # area already mapped? pick a new one
                g2, res2, ox2, oy2 = ex.grid()
                c = int((gx - ox2) / res2), int((gy - oy2) / res2)
                if 0 <= c[1] < g2.shape[0] and 0 <= c[0] < g2.shape[1]:
                    r0, r1 = max(0, c[1] - 3), c[1] + 4
                    q0, q1 = max(0, c[0] - 3), c[0] + 4
                    if not (g2[r0:r1, q0:q1] == -1).any():
                        ex.nav.cancelTask()
                        result = 'mapped'
                        break
        if result is None:
            r = ex.nav.getResult()
            result = 'reached' if r == TaskResult.SUCCEEDED else 'failed'
        while not ex.nav.isTaskComplete():       # make sure the cancel finished
            ex.spin(0.1)
        if result in ('reached', 'mapped'):
            reached += 1
            streak, last_good = [], (ex.robot_xy() or (gx, gy))
        else:
            failed += 1
            key = next((k for k in fails if math.hypot(k[0] - gx, k[1] - gy) < BLACKLIST_R), (gx, gy))
            fails[key] = (fails.get(key, (0, 0))[0] + 1, time.monotonic())
            streak.append(key)
        print(f'      {result}')
        if len(streak) >= 3:
            # three different goals failed in a row: the robot is probably wedged somewhere,
            # not the frontiers. Forgive those failures and go back to the last good spot.
            print(f'      3 failures in a row: returning to last good position '
                  f'({last_good[0]:.1f}, {last_good[1]:.1f})')
            for k in streak:
                cnt, _ = fails.get(k, (1, 0))
                fails[k] = (max(0, cnt - 1), 0.0)
            streak = []
            ex.nav.goToPose(ex.goal(*last_good))
            t_back = time.monotonic()
            while not ex.nav.isTaskComplete():
                ex.spin(0.2)
                if time.monotonic() - t_back > GOAL_TIMEOUT:
                    ex.nav.cancelTask()
                    break

    total = time.monotonic() - t0
    area = ex.explored_m2()
    print('\nExploration finished, returning home ...')
    ex.nav.goToPose(ex.goal(*home))
    while not ex.nav.isTaskComplete():
        ex.spin(0.2)
    log.append((round(time.monotonic() - t0, 1), round(ex.explored_m2(), 1), reached, failed, 0))

    csv_path = os.path.join(out, f'explore_{stamp}.csv')
    with open(csv_path, 'w', newline='') as f:
        csv.writer(f).writerows(log)
    print('=' * 60)
    print(f'Explored {area:.0f} m2 in {total / 60:.1f} min: '
          f'{reached} frontier goals reached, {failed} skipped')
    print(f'Results: {csv_path}')
    map_base = os.path.join(out, f'explore_map_{stamp}')
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base,
                    '--ros-args', '-p', 'use_sim_time:=true'], timeout=60, check=False)
    print(f'Map saved: {map_base}.pgm / .yaml')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
chmod +x $WS/scripts/ares_explore.py
/usr/bin/python3 -m py_compile $WS/scripts/ares_explore.py && echo "[2/3] scripts/ares_explore.py written"

# 3. rebuild
cd $WS && colcon build --symlink-install --packages-select ares_navigation
echo "[3/3] Done."
echo "  Terminal 1:  ros2 launch ares_navigation nav.launch.py"
echo "  Terminal 2:  python3 ~/ares_ws/scripts/ares_explore.py"
