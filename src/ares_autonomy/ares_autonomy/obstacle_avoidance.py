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
         /e_stop (std_msgs/Bool, overrides everything), cmd_in (driver commands)
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


def speed_scale(clearance, stop_dist, slow_dist):
    return float(np.clip((clearance - stop_dist) / (slow_dist - stop_dist), 0.0, 1.0))


def pick_heading(headings, free, prev, stop_dist, straight_w=0.6, smooth_w=0.3):
    """Best heading = lots of free space, small turn, close to last choice."""
    score = free - straight_w * np.abs(headings) - smooth_w * np.abs(headings - prev)
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
            'stop_dist': 0.25,       # closer than this: no motion towards the obstacle
            'slow_dist': 0.80,       # closer than this: speed scaled down linearly
            'auto_speed': 0.25, 'turn_gain': 1.5, 'max_turn': 1.2,
            'look_ahead': 3.0, 'sector_deg': 120.0, 'bins': 41,
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
        self.create_subscription(Range, 'ultrasonic/front', self.on_us, q)
        self.create_subscription(Twist, g('cmd_in'), self.on_cmd, 10)
        self.create_subscription(String, 'avoidance/mode', self.on_mode, 10)
        self.create_subscription(Bool, 'e_stop', lambda m: setattr(self, 'e_stop', m.data), 10)
        self.pub = self.create_publisher(Twist, g('cmd_out'), 10)
        self.pub_status = self.create_publisher(String, 'avoidance/status', 10)
        self.create_timer(0.05, self.loop)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"obstacle avoidance ready, mode={self.mode}, "
                               f"in={g('cmd_in')} out={g('cmd_out')}")

    # ---------- inputs ----------
    def on_scan(self, m):
        xs, ys = scan_to_xy(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)
        fr = corridor_free(xs, ys, [0.0, math.pi], self.hw, 0.0, self.look + 1)
        self.front_clear = max(0.0, fr[0] - self.f_off)
        self.rear_clear = max(0.0, fr[1] - self.r_off)
        self.free = corridor_free(xs, ys, self.headings, self.hw, self.f_off, self.look)
        ang = np.arctan2(ys, xs)
        d = np.hypot(xs, ys)
        left, right = (ang > 0.5) & (ang < 2.6), (ang < -0.5) & (ang > -2.6)
        self.left_space = float(np.mean(np.minimum(d[left], self.look))) if left.any() else self.look
        self.right_space = float(np.mean(np.minimum(d[right], self.look))) if right.any() else self.look
        self.scan_t = time.monotonic()

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
        if limited and not self.was_limited:
            self.interventions += 1
        self.was_limited = limited
        self.scale = s
        if abs(lin) > 0.01:
            self.state = 'clear' if s >= 1.0 else ('slowing' if s > 0 else 'blocked')
        return lin * s, ang

    def explore(self):
        now = time.monotonic()
        if now < self.recover_until:
            self.state = 'recovering'
            return (-0.12 if self.rear_clear > self.stop else 0.0), 0.6 * self.turn_dir
        fc = self.front_effective()
        heading, _ = pick_heading(self.headings, self.free, self.prev_heading, self.stop)
        if heading is None or fc < self.stop:
            if self.turn_dir == 0:                   # choose once, keep it (no dithering)
                self.turn_dir = 1 if self.left_space >= self.right_space else -1
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > 6.0:     # spun too long: back up, try again
                self.recover_until, self.blocked_since = now + 1.5, None
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
             'min_clearance_seen_m': round(self.min_clear_seen, 2)}
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
