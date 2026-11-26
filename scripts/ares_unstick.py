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
