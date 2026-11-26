#!/usr/bin/env bash
# ARES Part B: object-aware obstacle avoidance + one launch file for everything.
set -e
WS=$HOME/ares_ws
P=$WS/src/ares_autonomy
echo "=== ARES object-aware avoidance update ==="
[ "$(which python3)" = "/usr/bin/python3" ] || { echo "ERROR: run ares_env first"; exit 1; }
[ -d $P ] || { echo "ERROR: run ares_avoidance_setup.sh first"; exit 1; }

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


def parse_keepout(spec):
    """'person:0.8,car:0.6,default:0.3' -> dict"""
    out = {}
    for part in spec.split(','):
        if ':' in part:
            k, v = part.split(':', 1)
            out[k.strip()] = float(v)
    out.setdefault('default', 0.3)
    return out


def keepout_points(objects, radii, n=16):
    """Circle of points around each detected object (cls, bearing_rad, dist_m)."""
    xs, ys = [], []
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    for cls, b, d in objects:
        r = radii.get(cls, radii['default'])
        cx, cy = d * math.cos(b), d * math.sin(b)
        xs.append(cx + r * np.cos(t))
        ys.append(cy + r * np.sin(t))
    if not xs:
        return np.empty(0), np.empty(0)
    return np.concatenate(xs), np.concatenate(ys)


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
            'use_detections': True,
            'keepout': 'person:0.8,dog:0.6,cat:0.5,car:0.6,truck:0.6,bus:0.6,'
                       'bicycle:0.5,motorcycle:0.5,default:0.3',
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
        self.create_subscription(String, 'detections/objects', self.on_objects, 10)
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
        if self.use_det and time.monotonic() - self.obj_t < 1.0 and self.objects:
            vx, vy = keepout_points(self.objects, self.radii)
            xs, ys = np.concatenate([xs, vx]), np.concatenate([ys, vy])
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

    def on_objects(self, m):
        try:
            objs = json.loads(m.data).get('objects', [])
        except (ValueError, AttributeError):
            return
        self.objects = [(o['cls'], math.radians(o['bearing_deg']), o['dist_m'])
                        for o in objs if o.get('dist_m') is not None]
        self.obj_t = time.monotonic()

    def person_ahead(self):
        if not self.use_det or time.monotonic() - self.obj_t > 1.0:
            return False
        return any(c == 'person' and abs(b) < 0.5 and d < self.person_stop
                   for c, b, d in self.objects)

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
             'min_clearance_seen_m': round(self.min_clear_seen, 2),
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
echo "[1/3] obstacle_avoidance.py updated (object-aware)"

cat > $P/launch/avoid.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sim(context):
    gz = get_package_share_directory('ares_gazebo')
    world = LaunchConfiguration('world').perform(context)
    f = os.path.join(gz, 'launch', 'recon.launch.py' if world == 'recon' else 'sim.launch.py')
    if not os.path.exists(f):
        f = os.path.join(gz, 'launch', 'sim.launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(f))]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ares_autonomy'), 'config', 'avoidance.yaml')
    avoid = Node(package='ares_autonomy', executable='obstacle_avoidance', output='screen',
                 parameters=[cfg, {'mode': LaunchConfiguration('mode')}])
    detector = Node(package='ares_perception', executable='yolo_detector', output='screen',
                    condition=IfCondition(LaunchConfiguration('detect')))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='recon', description='recon or room'),
        DeclareLaunchArgument('mode', default_value='guard', description='guard, auto or off'),
        DeclareLaunchArgument('detect', default_value='true', description='run YOLO detector'),
        OpaqueFunction(function=_sim),
        avoid,
        detector,
    ])
EOF

cat > $P/config/avoidance.yaml << 'EOF'
obstacle_avoidance:
  ros__parameters:
    mode: guard            # guard | auto | off
    cmd_in: cmd_vel_raw    # driver commands come in here
    cmd_out: cmd_vel       # safe commands go to the wheels
    cmd_timeout: 0.0       # 0 = keep last command (keyboard style)
    stop_dist: 0.25        # m, no motion towards an obstacle closer than this
    slow_dist: 0.80        # m, start slowing down from here
    auto_speed: 0.25       # m/s, cruising speed in auto mode
    max_turn: 1.2          # rad/s
    use_detections: true   # use YOLO objects as keep-out zones
    keepout: "person:0.8,dog:0.6,cat:0.5,car:0.6,truck:0.6,bus:0.6,bicycle:0.5,motorcycle:0.5,default:0.3"
    person_stop_dist: 1.2  # m, auto mode stops for a person this close in front
    person_wait: 5.0       # s, then it goes around
EOF
grep -q "<exec_depend>ares_perception</exec_depend>" $P/package.xml || \
  sed -i 's|<exec_depend>ares_gazebo</exec_depend>|<exec_depend>ares_gazebo</exec_depend>\n  <exec_depend>ares_perception</exec_depend>|' $P/package.xml
echo "[2/3] launch + config updated"

cd $WS
/usr/bin/python3 -m py_compile $P/ares_autonomy/obstacle_avoidance.py && echo "avoidance syntax OK"
colcon build --symlink-install --packages-select ares_autonomy
echo "[3/3] Part B done. Launch: ros2 launch ares_autonomy avoid.launch.py"
