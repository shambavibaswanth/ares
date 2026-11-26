#!/usr/bin/env bash
# ARES self-rescue: the robot frees itself when stuck, during exploration and patrol.
# Also applies the Nav2 fixes: real robot shape, thinner wall padding, no turning at goals.
set -e
WS=$HOME/ares_ws
N=$WS/src/ares_navigation
P=$N/config/nav2_params.yaml
echo "=== ARES self-rescue setup ==="
[ "$(which python3)" = "/usr/bin/python3" ] || { echo "ERROR: run ares_env first"; exit 1; }
[ -f $P ] || { echo "ERROR: run ares_nav_setup.sh first"; exit 1; }

# 1. Nav2 settings (safe to run more than once)
sed -i 's/robot_radius: 0\.2[28]/footprint: "[[0.22, 0.20], [0.22, -0.20], [-0.22, -0.20], [-0.22, 0.20]]"/' $P
sed -i 's/inflation_radius: 0.55/inflation_radius: 0.40/g; s/cost_scaling_factor: 3.0/cost_scaling_factor: 4.0/g' $P
sed -i 's/yaw_goal_tolerance: 0.25/yaw_goal_tolerance: 6.28/' $P
echo "[1/4] Nav2: $(grep -c 'footprint: "' $P) footprints, $(grep -c 'inflation_radius: 0.40' $P) thin paddings, $(grep -c 'yaw_goal_tolerance: 6.28' $P) relaxed goal heading"

# 2. self-rescue module
cat > $WS/scripts/ares_unstick.py << 'EOF'
#!/usr/bin/env python3
"""ARES self-rescue: lets the robot find its own way out when Nav2 gets stuck.

Used by ares_explore.py and ares_patrol.py:
    un = Unsticker(nav)            # nav = BasicNavigator
    if un.stuck(): nav.cancelTask(); un.escape()

Stuck  = moved less than MIN_MOVE metres in the last WINDOW seconds (frozen or wiggling).
Escape = stop Nav2, look at the LiDAR, find the most open direction, make room if it is too
         tight to turn, turn towards the opening, drive out, clear Nav2's costmaps.
The robot drives itself during the escape (Nav2 is paused), using the LiDAR to stay safe.
"""
import math
import time
from collections import deque

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

WINDOW = 10.0        # s
MIN_MOVE = 0.25      # m
MIN_RANGE = 0.22     # m: closer LiDAR returns are the robot itself
LIDAR_X = 0.05       # m: LiDAR ahead of robot centre
HALF_LEN, HALF_WID = 0.20, 0.18
TURN_CLEAR = 0.32    # m from robot centre: corners can swing freely beyond this
SPEED, TURN = 0.12, 0.8


# ---------- pure helpers (no ROS, unit-testable) ----------
def scan_points(ranges, angle_min, inc):
    """LiDAR scan -> obstacle points (x, y) in the robot-centre frame."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * inc
    ok = np.isfinite(r) & (r >= MIN_RANGE)
    return r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])


def lane_clearance(px, py, heading, half_width=HALF_WID + 0.04, cap=4.0):
    """Free distance from the robot centre along `heading` in a robot-wide lane."""
    c, s = math.cos(heading), math.sin(heading)
    xp, yp = px * c + py * s, -px * s + py * c
    hit = (xp > 0) & (np.abs(yp) < half_width)
    return float(min(cap, xp[hit].min())) if hit.any() else cap


def most_open(px, py, step_deg=10):
    """(heading_rad, clearance_m) of the most open robot-wide lane, all 360 degrees.
    Small preference for not having to turn around."""
    heads = [math.radians(d) for d in range(-180, 180, step_deg)]
    clears = [lane_clearance(px, py, h) for h in heads]
    scores = [c + 0.15 * math.cos(h) for h, c in zip(heads, clears)]
    i = int(np.argmax(scores))
    return heads[i], clears[i]


def body_clearance(px, py, dx=0.0, dy=0.0, th=0.0):
    """Smallest gap between the robot rectangle (moved by dx, dy, th) and any point."""
    if len(px) == 0:
        return 9.0
    qx = math.cos(th) * (px - dx) + math.sin(th) * (py - dy)
    qy = -math.sin(th) * (px - dx) + math.cos(th) * (py - dy)
    ex, ey = np.abs(qx) - HALF_LEN, np.abs(qy) - HALF_WID
    out = np.hypot(np.maximum(ex, 0), np.maximum(ey, 0))
    return float(np.min(out + np.minimum(np.maximum(ex, ey), 0)))


def best_wriggle(px, py, secs=2.5):
    """Gentle arc (forward or back, steering either way) that moves the robot's centre
    furthest away from the nearest obstacle, i.e. creates room to turn. Like pulling a
    car away from a kerb, the tail may swing a little closer on the way, but the body
    never gets closer than 8 mm. None if no arc helps."""
    if len(px) == 0:
        return None
    floor = 0.008
    now = float(np.hypot(px, py).min())
    best, best_gain = None, 0.01
    for lin in (0.08, -0.08):
        for ang in (0.0, 0.15, -0.15, 0.3, -0.3, 0.5, -0.5):
            ok, dx, dy, th = True, 0.0, 0.0, 0.0
            for k in range(1, 11):
                t = secs * k / 10
                th = ang * t
                if abs(ang) > 1e-3:
                    dx, dy = lin / ang * math.sin(th), lin / ang * (1 - math.cos(th))
                else:
                    dx, dy = lin * t, 0.0
                if body_clearance(px, py, dx, dy, th) < floor:
                    ok = False
                    break
            if not ok:
                continue
            centre_gap = float(np.hypot(px - dx, py - dy).min())
            if centre_gap - now > best_gain:
                best, best_gain = (lin, ang), centre_gap - now
    return best


def nearest(px, py):
    return float(np.hypot(px, py).min()) if len(px) else 9.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Unsticker:
    def __init__(self, nav):
        self.nav = nav
        self.pub = nav.create_publisher(Twist, 'cmd_vel', 10)
        nav.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        nav.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.pts = (np.empty(0), np.empty(0))
        self.pose = None
        self.hist = deque()
        self.escapes = 0

    # ---------- inputs ----------
    def _on_scan(self, m):
        self.pts = scan_points(m.ranges, m.angle_min, m.angle_increment)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = time.monotonic()
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.hist.append((now, self.pose[0], self.pose[1]))
        while self.hist and now - self.hist[0][0] > WINDOW + 2:
            self.hist.popleft()

    # ---------- detection ----------
    def reset(self):
        """Call when a new goal starts, so old history does not count."""
        self.hist.clear()

    def stuck(self):
        if not self.hist or self.hist[-1][0] - self.hist[0][0] < WINDOW:
            return False
        t_now, x, y = self.hist[-1]
        recent = [(px, py) for t, px, py in self.hist if t_now - t <= WINDOW]
        return max(math.hypot(x - px, y - py) for px, py in recent) < MIN_MOVE

    # ---------- driving helpers ----------
    def _spin(self, secs):
        import rclpy
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.nav, timeout_sec=0.02)

    def _drive(self, lin, ang, secs, guard=None):
        """Publish (lin, ang) for secs; stop early if guard() returns False."""
        t = Twist()
        t.linear.x, t.angular.z = float(lin), float(ang)
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if guard is not None and not guard():
                break
            self.pub.publish(t)
            self._spin(0.05)
        self.pub.publish(Twist())
        self._spin(0.2)

    # ---------- the escape ----------
    def escape(self, log=print):
        self.escapes += 1
        self._spin(1.2)                       # let Nav2's last command time out
        px, py = self.pts
        heading, clear = most_open(px, py)
        log(f'      [escape {self.escapes}] most open: {math.degrees(heading):+.0f} deg, '
            f'{clear:.2f} m free; nearest obstacle {nearest(px, py):.2f} m')

        # 1. too tight to turn? wriggle (gentle arcs) until the corners can swing freely
        for _ in range(8):
            px, py = self.pts
            if nearest(px, py) >= TURN_CLEAR:
                break
            move = best_wriggle(px, py)
            if move is None:
                break
            log(f'      [escape] tight spot: arc lin {move[0]:+.2f} ang {move[1]:+.2f}')
            self._drive(*move, 2.5, guard=lambda: body_clearance(*self.pts) > 0.008)

        # 2. turn towards the opening (re-measured), stop if a corner gets too close
        px, py = self.pts
        heading, clear = most_open(px, py)
        if self.pose is not None and abs(heading) > math.radians(15):
            target = wrap(self.pose[2] + heading)
            sign = 1 if heading > 0 else -1
            t_end = time.monotonic() + abs(heading) / TURN + 3.0
            while time.monotonic() < t_end:
                err = wrap(target - self.pose[2])
                if abs(err) < math.radians(12):
                    break
                if nearest(*self.pts) < TURN_CLEAR - 0.08:
                    log('      [escape] corner too close, stopping the turn')
                    break
                self._drive(0.0, TURN * sign, 0.1)

        # 3. drive out along the opening
        px, py = self.pts
        ahead = lane_clearance(px, py, 0.0) - HALF_LEN
        if ahead > 0.25:
            dist = min(0.6, ahead - 0.15)
            log(f'      [escape] driving out {dist:.2f} m')
            self._drive(SPEED, 0.0, dist / SPEED,
                        guard=lambda: lane_clearance(*self.pts, 0.0) - HALF_LEN > 0.15)
        else:
            back = lane_clearance(px, py, math.pi) - HALF_LEN
            if back > 0.25:
                log('      [escape] no room ahead, backing out')
                self._drive(-SPEED, 0.0, min(0.5, back - 0.15) / SPEED,
                            guard=lambda: lane_clearance(*self.pts, math.pi) - HALF_LEN > 0.15)

        # 4. let Nav2 start fresh
        try:
            self.nav.clearAllCostmaps()
        except Exception:
            pass
        self.reset()
EOF

# 3. explorer + patrol, both using self-rescue
cat > $WS/scripts/ares_explore.py << 'EOF'
#!/usr/bin/env python3
"""ARES autonomous frontier exploration (SLAM Toolbox + Nav2).

The robot explores everywhere it can reach, with no waypoints given:
  1. read the live SLAM map (/map)
  2. find "frontiers": free cells next to unknown space, grouped into clusters
  3. drop frontiers too close to walls or ones that already failed
  4. send Nav2 to the best one (near, large); re-plan when that area becomes known
  5. when no frontiers remain: drive home, save the map, write results
If the robot gets stuck (frozen or wiggling for 10 s) it frees itself with ares_unstick.py:
it looks for the most open direction, wriggles out of tight spots, turns and drives clear.

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

from ares_unstick import Unsticker

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
        self.un = Unsticker(self.nav)

    def go(self, x, y, timeout=GOAL_TIMEOUT, max_escapes=2):
        """Drive to (x, y); free itself if stuck. Returns 'reached', 'failed' or 'timeout'."""
        for _ in range(max_escapes + 1):
            self.nav.goToPose(self.goal(x, y))
            self.un.reset()
            t0, stuck = time.monotonic(), False
            while not self.nav.isTaskComplete():
                if time.monotonic() - t0 > timeout:
                    self.nav.cancelTask()
                    self.finish()
                    return 'timeout'
                if self.un.stuck():
                    print('      stuck: cancelling goal and freeing myself')
                    self.nav.cancelTask()
                    self.finish()
                    self.un.escape()
                    stuck = True
                    break
            if not stuck:
                return 'reached' if self.nav.getResult() == TaskResult.SUCCEEDED else 'failed'
        return 'failed'

    def finish(self):
        while not self.nav.isTaskComplete():
            self.spin(0.1)

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
    log = [('time_s', 'explored_m2', 'goals_reached', 'goals_failed', 'frontiers_left', 'escapes')]
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
        log.append((round(time.monotonic() - t0, 1), round(ex.explored_m2(), 1), reached, failed, n,
                    ex.un.escapes))
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
        ex.un.reset()
        t_goal, result = time.monotonic(), None
        while not ex.nav.isTaskComplete():
            el = time.monotonic() - t_goal
            if el > GOAL_TIMEOUT:
                ex.nav.cancelTask()
                result = 'timeout'
                break
            if ex.un.stuck():                     # frozen or wiggling: free myself
                print('      stuck: cancelling goal and freeing myself')
                ex.nav.cancelTask()
                ex.finish()
                ex.un.escape()
                result = 'stuck'
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
            if result != 'stuck':                 # after an escape the robot is free again
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
            ex.go(*last_good)

    total = time.monotonic() - t0
    area = ex.explored_m2()
    print('\nExploration finished, returning home ...')
    print('  home:', ex.go(*home, timeout=180.0))
    log.append((round(time.monotonic() - t0, 1), round(ex.explored_m2(), 1), reached, failed, 0,
                ex.un.escapes))

    csv_path = os.path.join(out, f'explore_{stamp}.csv')
    with open(csv_path, 'w', newline='') as f:
        csv.writer(f).writerows(log)
    print('=' * 60)
    print(f'Explored {area:.0f} m2 in {total / 60:.1f} min: '
          f'{reached} frontier goals reached, {failed} skipped, {ex.un.escapes} self-rescues')
    print(f'Results: {csv_path}')
    map_base = os.path.join(out, f'explore_map_{stamp}')
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base,
                    '--ros-args', '-p', 'use_sim_time:=true'], timeout=60, check=False)
    print(f'Map saved: {map_base}.pgm / .yaml')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

cat > $WS/scripts/ares_patrol.py << 'EOF'
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
EOF
chmod +x $WS/scripts/ares_explore.py $WS/scripts/ares_patrol.py
cd $WS/scripts && /usr/bin/python3 -m py_compile ares_unstick.py ares_explore.py ares_patrol.py \
  && /usr/bin/python3 -c "import ares_unstick" && echo "[2/4] ares_unstick.py  [3/4] ares_explore.py + ares_patrol.py"

# 4. rebuild
cd $WS && colcon build --symlink-install --packages-select ares_navigation
echo "[4/4] Done."
echo "  Terminal 1:  ros2 launch ares_navigation nav.launch.py"
echo "  Terminal 2:  python3 ~/ares_ws/scripts/ares_explore.py     (or ares_patrol.py)"
