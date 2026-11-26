#!/usr/bin/env bash
# ARES avoidance v3: obstacles only matter when very near (12 cm), whole-body collision
# check, exploration memory so auto mode covers every reachable path, LiDAR self-hit fix.
set -e
WS=$HOME/ares_ws
P=$WS/src/ares_autonomy
echo "=== ARES avoidance v3 ==="
[ "$(which python3)" = "/usr/bin/python3" ] || { echo "ERROR: run ares_env first"; exit 1; }
[ -d $P ] || { echo "ERROR: ares_autonomy missing, run ares_avoidance_setup.sh first"; exit 1; }

# 1. LiDAR ignores its own robot body (returns closer than 0.22 m)
U=$WS/src/ares_description/urdf/ares.urdf.xacro
sed -i 's|<range><min>0.12</min><max>10.0</max>|<range><min>0.22</min><max>10.0</max>|' $U
grep -q "<min>0.22</min>" $U && echo "[1/4] LiDAR min range 0.22 m" || { echo "ERROR: LiDAR range line not found"; exit 1; }

# 2. new avoidance node
cat > $P/ares_autonomy/obstacle_avoidance.py << 'EOF'
#!/usr/bin/env python3
"""ARES obstacle avoidance: safety guard + autonomous explore mode.

Modes (switch while running):
    ros2 topic pub --once /avoidance/mode std_msgs/String "data: auto"
  guard : passes driver commands (cmd_in) to the wheels, but slows down near obstacles
          and refuses to drive into them, forwards or backwards. Turning stays allowed.
  auto  : drives by itself towards the widest free gap ahead (reactive explorer),
          rotates out of dead ends and backs up if stuck. The guard still applies.
  off   : passes commands straight through (baseline for comparison tests).

Inputs : /scan (360 deg LiDAR), /ultrasonic/front (low obstacles under the LiDAR),
         /detections/objects (YOLO + LiDAR distance, from ares_perception),
         /e_stop (std_msgs/Bool, overrides everything), cmd_in (driver commands)

Object-aware behaviour: every detected object becomes a "keep-out" circle added to
the LiDAR picture, sized by class (a person gets a much bigger margin than a box).
In auto mode the robot also stops and waits when a person is close in front of it,
then goes around if they stay.

Exploration memory: in auto mode the robot remembers which 0.5 m cells of the world it
has already driven through (from odometry) and prefers headings that lead to cells it
has not visited, so it works its way into every reachable corridor, room and doorway
instead of circling the same area. Explored area is reported in the status.
Output : cmd_out (wheels), /avoidance/status (JSON, 5 Hz)
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String

MODES = ('guard', 'auto', 'off')


# ---------- pure helpers (no ROS, unit-testable) ----------
def scan_to_xy(ranges, angle_min, angle_inc, rmin, rmax):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])


def corridor_free(xs, ys, headings, half_width, offset, cap):
    """Free distance along a robot-wide corridor for each heading (radians)."""
    out = np.full(len(headings), float(cap))
    for i, th in enumerate(headings):
        c, s = math.cos(th), math.sin(th)
        xp = xs * c + ys * s
        yp = -xs * s + ys * c
        hit = (xp > 0) & (np.abs(yp) < half_width)
        if hit.any():
            out[i] = min(cap, max(0.0, float(xp[hit].min()) - offset))
    return out


def parse_keepout(spec):
    """'person:0.8,car:0.6,default:0.3' -> dict"""
    out = {}
    for part in spec.split(','):
        if ':' in part:
            k, v = part.split(':', 1)
            out[k.strip()] = float(v)
    out.setdefault('default', 0.3)
    return out


def keepout_points(objects, radii, n=16, min_dist=0.4, robot_r=0.35):
    """Circle of points around each detected object (cls, bearing_rad, dist_m).
    Objects closer than min_dist are ignored (LiDAR already covers them); points
    that would land on the robot itself (within robot_r of the LiDAR) are dropped."""
    xs, ys = [], []
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    for cls, b, d in objects:
        if d is None or d < min_dist:
            continue
        r = radii.get(cls, radii['default'])
        cx, cy = d * math.cos(b), d * math.sin(b)
        xs.append(cx + r * np.cos(t))
        ys.append(cy + r * np.sin(t))
    if not xs:
        return np.empty(0), np.empty(0)
    x, y = np.concatenate(xs), np.concatenate(ys)
    keep = np.hypot(x, y) > robot_r
    return x[keep], y[keep]


def body_clearance(px, py, dx=0.0, dy=0.0, th=0.0, half_len=0.2, half_wid=0.18):
    """Smallest distance from any obstacle point to the robot rectangle placed at
    (dx, dy, th) relative to where it is now (negative = point inside the body)."""
    if len(px) == 0:
        return 9.0
    qx = math.cos(th) * (px - dx) + math.sin(th) * (py - dy)
    qy = -math.sin(th) * (px - dx) + math.cos(th) * (py - dy)
    ex, ey = np.abs(qx) - half_len, np.abs(qy) - half_wid
    outside = np.hypot(np.maximum(ex, 0), np.maximum(ey, 0))
    inside = np.minimum(np.maximum(ex, ey), 0)
    return float(np.min(outside + inside))


def footprint_hit(px, py, lin, ang, margin=0.03, horizon=0.4):
    """Would following (lin, ang) for `horizon` s bring the robot body closer than
    `margin` to an obstacle point? Moves that only increase an already-too-small
    clearance are allowed, so the robot can always work its way out of a tight spot."""
    if len(px) == 0:
        return False
    limit = min(margin, body_clearance(px, py)) - 1e-3
    for frac in (0.25, 0.5, 0.75, 1.0):
        t = horizon * frac
        th = ang * t
        if abs(ang) > 1e-3:
            dx, dy = lin / ang * math.sin(th), lin / ang * (1 - math.cos(th))
        else:
            dx, dy = lin * t, 0.0
        if body_clearance(px, py, dx, dy, th) < limit:
            return True
    return False


def speed_scale(clearance, stop_dist, slow_dist):
    return float(np.clip((clearance - stop_dist) / (slow_dist - stop_dist), 0.0, 1.0))


def pick_heading(headings, free, prev, stop_dist, straight_w=0.3, smooth_w=0.3, penalty=None):
    """Best heading = lots of free space, small turn, close to last choice,
    and (if a penalty is given) leading towards places not visited yet."""
    score = free - straight_w * np.abs(headings) - smooth_w * np.abs(headings - prev)
    if penalty is not None:
        score = score - penalty
    score[free <= stop_dist + 0.1] = -1e9
    i = int(np.argmax(score))
    return (None if score[i] < -1e8 else float(headings[i])), i


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        defaults = {
            'mode': 'guard', 'cmd_in': 'cmd_vel_raw', 'cmd_out': 'cmd_vel',
            'cmd_timeout': 0.0,      # 0 = hold last command (keyboard); 0.5 when fed by a mux
            'half_width': 0.22,      # half robot width incl. wheels + margin (m)
            'front_offset': 0.15,    # LiDAR -> front bumper (m)
            'rear_offset': 0.25,     # LiDAR -> rear bumper (m)
            'stop_dist': 0.12,       # closer than this (from the bumper): no motion towards it
            'slow_dist': 0.45,       # closer than this: speed scaled down linearly
            'auto_speed': 0.25, 'turn_gain': 1.5, 'max_turn': 1.2,
            'look_ahead': 3.0, 'sector_deg': 120.0, 'bins': 41,
            'use_detections': True,
            'keepout': 'person:0.5,dog:0.4,cat:0.3,car:0.3,truck:0.3,bus:0.3,'
                       'bicycle:0.3,motorcycle:0.3,default:0.15',
            'cell_size': 0.5,         # exploration memory grid (m)
            'novelty_weight': 0.8,    # how strongly auto mode prefers unvisited places
            'probe_dist': 1.5,        # how far ahead each heading is checked for novelty
            'spin_clear': 0.33,       # m from LiDAR: turning in place is only safe beyond this
            'explore_half_width': 0.32,  # auto mode keeps this much side room (space to turn)
            'min_range': 0.22,        # m: LiDAR returns closer than this are the robot itself
            'lidar_x': 0.05,          # LiDAR position ahead of the robot centre (m)
            'person_stop_dist': 1.2,  # auto mode: stop if a person is this close in front
            'person_wait': 5.0,       # ...and wait this long before going around
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.mode = g('mode') if g('mode') in MODES else 'guard'
        self.cmd_timeout = float(g('cmd_timeout'))
        self.hw, self.f_off, self.r_off = g('half_width'), g('front_offset'), g('rear_offset')
        self.stop, self.slow = g('stop_dist'), g('slow_dist')
        self.auto_speed, self.gain, self.max_turn = g('auto_speed'), g('turn_gain'), g('max_turn')
        self.look = g('look_ahead')
        half = math.radians(g('sector_deg')) / 2
        self.headings = np.linspace(-half, half, int(g('bins')))
        self.use_det = bool(g('use_detections'))
        self.radii = parse_keepout(g('keepout'))
        self.person_stop, self.person_wait = g('person_stop_dist'), g('person_wait')
        self.objects, self.obj_t, self.person_since = [], 0.0, None
        self.cell, self.nov_w, self.probe = g('cell_size'), g('novelty_weight'), g('probe_dist')
        self.visits = {}
        self.spin_clear = g('spin_clear')
        self.min_range, self.lidar_x = g('min_range'), g('lidar_x')
        self.ehw = g('explore_half_width')
        self.free_wide = np.full(len(self.headings), self.look)
        self.pts = (np.empty(0), np.empty(0))   # obstacle points, robot-centre frame
        self.footprint_blocks = 0
        self.nearest = self.look             # nearest LiDAR point in any direction
        self.progress = None                 # (x, y, t) watchdog reference
        self.escape_until, self.escape_turn_from = 0.0, 0.0
        self.escapes = 0
        self.last_new_cell = 0.0             # time a never-visited cell was last entered
        self.pose = None                     # (x, y, yaw) from odometry

        self.cmd, self.cmd_t = (0.0, 0.0), 0.0
        self.scan_t, self.us_t = 0.0, 0.0
        self.front_clear = self.rear_clear = self.look
        self.us_front = self.look
        self.free = np.full(len(self.headings), self.look)
        self.left_space = self.right_space = self.look
        self.e_stop = False
        self.prev_heading, self.turn_dir = 0.0, 0
        self.blocked_since, self.recover_until = None, 0.0
        self.state, self.scale = 'starting', 1.0
        self.interventions, self.was_limited = 0, False
        self.min_clear_seen = self.look

        q = qos_profile_sensor_data
        self.create_subscription(LaserScan, 'scan', self.on_scan, q)
        self.create_subscription(Range, 'ultrasonic/front', self.on_us_raw, q, raw=True)
        self.us_errors = 0
        self.create_subscription(Twist, g('cmd_in'), self.on_cmd, 10)
        self.create_subscription(String, 'avoidance/mode', self.on_mode, 10)
        self.create_subscription(String, 'detections/objects', self.on_objects, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, q)
        self.create_subscription(Bool, 'e_stop', lambda m: setattr(self, 'e_stop', m.data), 10)
        self.pub = self.create_publisher(Twist, g('cmd_out'), 10)
        self.pub_status = self.create_publisher(String, 'avoidance/status', 10)
        self.create_timer(0.05, self.loop)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"obstacle avoidance ready, mode={self.mode}, "
                               f"in={g('cmd_in')} out={g('cmd_out')}")

    # ---------- inputs ----------
    def on_scan(self, m):
        xs, ys = scan_to_xy(m.ranges, m.angle_min, m.angle_increment,
                            max(m.range_min, self.min_range), m.range_max)
        if self.use_det and time.monotonic() - self.obj_t < 1.0 and self.objects:
            vx, vy = keepout_points(self.objects, self.radii)
            xs, ys = np.concatenate([xs, vx]), np.concatenate([ys, vy])
        fr = corridor_free(xs, ys, [0.0, math.pi], self.hw, 0.0, self.look + 1)
        self.front_clear = max(0.0, fr[0] - self.f_off)
        self.rear_clear = max(0.0, fr[1] - self.r_off)
        self.free = corridor_free(xs, ys, self.headings, self.hw, self.f_off, self.look)
        self.free_wide = corridor_free(xs, ys, self.headings, self.ehw, self.f_off, self.look)
        ang = np.arctan2(ys, xs)
        d = np.hypot(xs, ys)
        self.nearest = float(d.min()) if len(d) else self.look
        near = d < 1.0
        self.pts = (xs[near] + self.lidar_x, ys[near])
        left, right = (ang > 0.5) & (ang < 2.6), (ang < -0.5) & (ang > -2.6)
        self.left_space = float(np.mean(np.minimum(d[left], self.look))) if left.any() else self.look
        self.right_space = float(np.mean(np.minimum(d[right], self.look))) if right.any() else self.look
        self.scan_t = time.monotonic()

    def on_objects(self, m):
        try:
            objs = json.loads(m.data).get('objects', [])
        except (ValueError, AttributeError):
            return
        self.objects = [(o['cls'], math.radians(o['bearing_deg']), o['dist_m'])
                        for o in objs if o.get('dist_m') is not None]
        self.obj_t = time.monotonic()

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def cell_of(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def novelty_penalty(self):
        """Per heading: how much the robot has already been where that heading leads."""
        if self.pose is None or self.nov_w <= 0:
            return None
        x, y, yaw = self.pose
        pen = np.zeros(len(self.headings))
        for i, h in enumerate(self.headings):
            d = min(float(self.free_wide[i]), self.probe)
            c = self.cell_of(x + d * math.cos(yaw + h), y + d * math.sin(yaw + h))
            pen[i] = self.nov_w * min(self.visits.get(c, 0) / 40.0, 3.0)   # 40 ticks = 2 s
        return pen

    def person_ahead(self):
        if not self.use_det or time.monotonic() - self.obj_t > 1.0:
            return False
        return any(c == 'person' and abs(b) < 0.5 and d < self.person_stop
                   for c, b, d in self.objects)

    def on_us_raw(self, data):
        try:
            m = deserialize_message(data, Range)
        except Exception:                      # undecodable Range message: ignore it
            self.us_errors += 1
            if self.us_errors == 1:
                self.get_logger().warn('ultrasonic message could not be decoded; using LiDAR only')
            return
        self.on_us(m)

    def on_us(self, m):
        self.us_front = m.range if math.isfinite(m.range) else self.look
        self.us_t = time.monotonic()

    def on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), time.monotonic()

    def on_mode(self, m):
        mode = m.data.strip().lower()
        if mode not in MODES:
            self.get_logger().warn(f'unknown mode "{mode}", use one of {MODES}')
            return
        self.mode, self.cmd = mode, (0.0, 0.0)
        self.turn_dir, self.blocked_since, self.recover_until = 0, None, 0.0
        self.get_logger().info(f'mode -> {mode}')

    # ---------- behaviour ----------
    def front_effective(self):
        fc = self.front_clear
        if time.monotonic() - self.us_t < 0.5:      # ultrasonic sits at the bumper
            fc = min(fc, self.us_front)
        return fc

    def guard(self, lin, ang):
        if lin > 0:
            s = speed_scale(self.front_effective(), self.stop, self.slow)
        elif lin < 0:
            s = speed_scale(self.rear_clear, self.stop, self.slow)
        else:
            s = 1.0
        limited = abs(lin) > 0.01 and s < 1.0
        self.scale = s
        if abs(lin) > 0.01:
            self.state = 'clear' if s >= 1.0 else ('slowing' if s > 0 else 'blocked')
        lin = lin * s
        # whole-body check: never let a corner or side swing into something
        px, py = self.pts
        if footprint_hit(px, py, lin, ang):
            limited = True
            self.footprint_blocks += 1
            cands = [(lin, 0.0), (0.0, ang)]
            if self.mode == 'auto':                  # auto may wriggle out on its own
                spin = ang if abs(ang) > 0.1 else 0.8
                cands += [(0.0, -spin), (0.06, 0.4), (0.06, -0.4), (-0.06, 0.4), (-0.06, -0.4),
                          (-0.08, 0.0), (-0.08, spin), (-0.08, -spin)]
            cands.append((0.0, 0.0))
            for cand in cands:
                if cand == (0.0, 0.0) or not footprint_hit(px, py, *cand):
                    lin, ang = cand
                    break
            if (lin, ang) == (0.0, 0.0):
                self.state = 'blocked'
        if limited and not self.was_limited:
            self.interventions += 1
        self.was_limited = limited
        return lin, ang

    def check_progress(self, now):
        """Watchdog: no real movement for 8 s -> back out and turn somewhere new."""
        if self.pose is None:
            return
        x, y, _ = self.pose
        if self.progress is None or now < self.escape_until:
            self.progress = (x, y, now)
            return
        px, py, pt = self.progress
        if math.hypot(x - px, y - py) > 0.4:
            self.progress = (x, y, now)
        stalled = now - self.progress[2] > 8.0              # not moving
        looping = now - self.last_new_cell > 25.0           # moving, but nowhere new
        if stalled or looping:
            self.last_new_cell = now
            self.escapes += 1
            pen = self.novelty_penalty()
            left = self.left_space - (pen[-1] if pen is not None else 0)
            right = self.right_space - (pen[0] if pen is not None else 0)
            self.turn_dir = 1 if left >= right else -1
            if self.escapes % 3 == 0:            # every third escape, try the other way
                self.turn_dir = -self.turn_dir
            self.escape_turn_from = now + 1.2    # 1.2 s reverse, then turn
            self.escape_until = now + 1.2 + 1.5 + 0.5 * (self.escapes % 3)
            self.progress = (x, y, now)
            self.get_logger().info('stuck or looping: escaping')

    def explore(self):
        now = time.monotonic()
        self.check_progress(now)
        if now < self.escape_until:
            self.state = 'escaping'
            if now < self.escape_turn_from and self.rear_clear > self.stop:
                return -0.12, 0.0
            return 0.0, 0.8 * self.max_turn * self.turn_dir
        if now < self.recover_until:
            self.state = 'recovering'
            return (-0.12 if self.rear_clear > self.stop else 0.0), 0.6 * self.turn_dir
        if self.person_ahead():
            if self.person_since is None:
                self.person_since = now
                self.get_logger().info('person ahead: stopping and waiting')
            if now - self.person_since < self.person_wait:
                self.state = 'waiting_for_person'
                return 0.0, 0.0
        else:
            self.person_since = None
        fc = self.front_effective()
        heading, _ = pick_heading(self.headings, self.free_wide, self.prev_heading, self.stop,
                                  penalty=self.novelty_penalty())
        if heading is None or fc < self.stop:
            if self.turn_dir == 0:                   # choose once, keep it (no dithering)
                self.turn_dir = 1 if self.left_space >= self.right_space else -1
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > 6.0:     # spun too long: back up, try again
                self.recover_until, self.blocked_since = now + 1.5, None
            if self.nearest < self.spin_clear and self.rear_clear > self.stop:
                self.state = 'backing_out'           # too tight to spin: reverse first
                return -0.1, 0.0
            self.state = 'turning'
            return 0.0, 0.7 * self.max_turn * self.turn_dir
        self.turn_dir, self.blocked_since = 0, None
        self.prev_heading = heading
        ang = float(np.clip(self.gain * heading, -self.max_turn, self.max_turn))
        lin = self.auto_speed * max(0.3, speed_scale(fc, self.stop, self.slow)) * max(0.0, math.cos(heading))
        self.state = 'exploring'
        return lin, ang

    def loop(self):
        now = time.monotonic()
        self.min_clear_seen = min(self.min_clear_seen, self.front_effective())
        if self.pose is not None:
            c = self.cell_of(self.pose[0], self.pose[1])
            if c not in self.visits:
                self.last_new_cell = now
            self.visits[c] = self.visits.get(c, 0) + 1
        if self.e_stop:
            self.state, out = 'e_stop', (0.0, 0.0)
        elif self.mode != 'off' and now - self.scan_t > 0.5:
            self.state, out = 'no_lidar', (0.0, 0.0)          # blind: do not move
        elif self.mode == 'off':
            self.state, out = 'off', self.cmd
        else:
            if self.mode == 'auto':
                lin, ang = self.explore()
            else:
                lin, ang = self.cmd
                if self.cmd_timeout > 0 and now - self.cmd_t > self.cmd_timeout:
                    lin, ang = 0.0, 0.0
                if abs(lin) < 0.01:
                    self.state = 'idle'
            out = self.guard(lin, ang)
        t = Twist()
        t.linear.x, t.angular.z = float(out[0]), float(out[1])
        self.pub.publish(t)

    def publish_status(self):
        s = {'mode': self.mode, 'state': self.state,
             'front_clear_m': round(self.front_effective(), 2),
             'rear_clear_m': round(self.rear_clear, 2),
             'speed_scale': round(self.scale, 2),
             'interventions': self.interventions,
             'min_clearance_seen_m': round(self.min_clear_seen, 2),
             'explored_m2': round(len(self.visits) * self.cell ** 2, 1),
             'escapes': self.escapes,
             'objects': [f'{c}@{d:.1f}m' for c, _, d in self.objects]
                        if time.monotonic() - self.obj_t < 1.0 else []}
        self.pub_status.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
echo "[2/4] obstacle_avoidance.py v3 written"

# 3. settings
cat > $P/config/avoidance.yaml << 'EOF'
obstacle_avoidance:
  ros__parameters:
    mode: guard              # guard | auto | off
    cmd_in: cmd_vel_raw      # driver commands come in here
    cmd_out: cmd_vel         # safe commands go to the wheels
    cmd_timeout: 0.0         # 0 = keep last command (keyboard style)
    stop_dist: 0.12          # m from the bumper: react only when very near
    slow_dist: 0.45          # m: start slowing down from here
    auto_speed: 0.25         # m/s cruising speed in auto mode
    max_turn: 1.2            # rad/s
    min_range: 0.22          # m: closer LiDAR returns are the robot itself
    explore_half_width: 0.32 # m: auto mode keeps room to turn around
    novelty_weight: 0.8      # how strongly auto mode prefers places it has not been
    cell_size: 0.5           # m: exploration memory grid
    use_detections: true     # YOLO objects become (small) keep-out zones
    keepout: "person:0.5,dog:0.4,cat:0.3,car:0.3,truck:0.3,bus:0.3,bicycle:0.3,motorcycle:0.3,default:0.15"
    person_stop_dist: 1.0    # m: auto mode pauses for a person this close in front
    person_wait: 5.0         # s: then goes around
EOF
echo "[3/4] settings written"

cd $WS
/usr/bin/python3 -m py_compile $P/ares_autonomy/obstacle_avoidance.py && echo "syntax OK"
colcon build --symlink-install --packages-select ares_description ares_autonomy
echo "[4/4] Done. Start with:  ~/ares_ws/scripts/ares_run.sh auto"
