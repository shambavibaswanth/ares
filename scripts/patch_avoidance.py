import os, re, sys
f = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/ares_ws/src/ares_autonomy/ares_autonomy/obstacle_avoidance.py')
s = open(f).read()
changed = []
# 1) ultrasonic: take raw bytes and decode ourselves, so a bad Range message can never kill the node
if 'on_us_raw' not in s:
    s = s.replace("from rclpy.qos import qos_profile_sensor_data\n",
                  "from rclpy.qos import qos_profile_sensor_data\nfrom rclpy.serialization import deserialize_message\n", 1)
    s = s.replace("self.create_subscription(Range, 'ultrasonic/front', self.on_us, q)",
                  "self.create_subscription(Range, 'ultrasonic/front', self.on_us_raw, q, raw=True)\n        self.us_errors = 0", 1)
    s = s.replace("    def on_us(self, m):",
'''    def on_us_raw(self, data):
        try:
            m = deserialize_message(data, Range)
        except Exception:                      # undecodable Range message: ignore it
            self.us_errors += 1
            if self.us_errors == 1:
                self.get_logger().warn('ultrasonic message could not be decoded; using LiDAR only')
            return
        self.on_us(m)

    def on_us(self, m):''', 1)
    changed.append('ultrasonic')
# 2) keep-out fix (ignore impossible close detections), in case it was not applied yet
if 'min_dist=0.4' not in s:
    new = '''def keepout_points(objects, radii, n=16, min_dist=0.4, robot_r=0.35):
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


'''
    s = re.sub(r'def keepout_points\(.*?\n(?=def speed_scale)', new, s, count=1, flags=re.S)
    changed.append('keep-out')
assert 'on_us_raw' in s and 'min_dist=0.4' in s, 'patch failed'
open(f, 'w').write(s)
print('patched:', ', '.join(changed) if changed else 'already up to date')
