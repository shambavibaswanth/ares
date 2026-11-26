#!/usr/bin/env bash
# ARES fix-all: clean ares_env, offline-safe ROS, apply avoidance patch, check files, rebuild.
WS=$HOME/ares_ws
ok()   { echo "  [OK]   $1"; }
bad()  { echo "  [FIX]  $1"; PROBLEMS=$((PROBLEMS+1)); }
PROBLEMS=0
echo "=== ARES fix-all ==="

# 1. one clean ares_env in ~/.bashrc (removes old copies)
sed -i '/^ares_env() {/,/^}/d' ~/.bashrc
cat >> ~/.bashrc << 'EOF'
ares_env() {
  unset VIRTUAL_ENV
  export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'omnetpp-6.4.0/.venv/bin' | paste -sd:)
  hash -r
  export ROS_LOCALHOST_ONLY=1
  export GAZEBO_IP=127.0.0.1
  source /opt/ros/humble/setup.bash
  [ -f ~/ares_ws/install/setup.bash ] && source ~/ares_ws/install/setup.bash
  export GAZEBO_MODEL_PATH=$(echo "$GAZEBO_MODEL_PATH" | tr ':' '\n' | grep -v urdf_tutorial | paste -sd:)
}
EOF
[ "$(grep -c '^ares_env() {' ~/.bashrc)" = "1" ] && ok "ares_env set up (offline-safe)" || bad "ares_env"
unset VIRTUAL_ENV
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'omnetpp-6.4.0/.venv/bin' | paste -sd:)
hash -r
export ROS_LOCALHOST_ONLY=1 GAZEBO_IP=127.0.0.1
source /opt/ros/humble/setup.bash
[ "$(which python3)" = "/usr/bin/python3" ] && ok "python3 = /usr/bin/python3" || bad "python3 is $(which python3)"

# 2. stop anything left over
pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null; pkill -9 -f obstacle_avoidance 2>/dev/null
pkill -9 -f yolo_detector 2>/dev/null; pkill -9 -f teleop_twist 2>/dev/null
ros2 daemon stop > /dev/null 2>&1
ok "old processes stopped"

# 3. required pieces
for f in src/ares_gazebo/worlds/recon_site.world src/ares_gazebo/launch/recon.launch.py \
         src/ares_autonomy/ares_autonomy/obstacle_avoidance.py src/ares_autonomy/launch/avoid.launch.py \
         src/ares_perception/ares_perception/yolo_detector.py models/yolov8n.pt; do
  [ -f "$WS/$f" ] && ok "$f" || bad "MISSING $f"
done

# 4. avoidance patch (ultrasonic crash + keep-out fix)
AV=$WS/src/ares_autonomy/ares_autonomy/obstacle_avoidance.py
if [ -f "$AV" ]; then
cat > $WS/scripts/patch_avoidance.py << 'PYEOF'
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
PYEOF
  /usr/bin/python3 $WS/scripts/patch_avoidance.py "$AV" && ok "avoidance patch applied" || bad "avoidance patch"
fi

# 5. launch helper that hides the multicast spam
cat > $WS/scripts/ares_run.sh << 'EOF'
#!/usr/bin/env bash
# Usage: ares_run.sh [guard|auto]   (full log also saved in ~/ares_ws/results/logs/)
source /opt/ros/humble/setup.bash
source ~/ares_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 GAZEBO_IP=127.0.0.1
pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null; sleep 1
mkdir -p ~/ares_ws/results/logs
LOG=~/ares_ws/results/logs/run_$(date +%Y%m%d_%H%M%S).log
echo "Starting ARES (mode=${1:-guard}). Full log: $LOG"
ros2 launch ares_autonomy avoid.launch.py mode:=${1:-guard} 2>&1 | tee "$LOG" | \
  grep --line-buffered -v -E "multicast|Missing model.config|noise index|Deleting a connection|Queue limit"
EOF
chmod +x $WS/scripts/ares_run.sh
ok "scripts/ares_run.sh ready"

# 6. rebuild everything
cd $WS
if colcon build --symlink-install > /tmp/ares_build.log 2>&1; then
  ok "build: $(grep -o 'Summary: .*' /tmp/ares_build.log)"
else
  bad "build failed:"; tail -20 /tmp/ares_build.log
fi
source $WS/install/setup.bash
for p in ares_gazebo ares_autonomy ares_perception; do
  ros2 pkg prefix $p > /dev/null 2>&1 && ok "package $p" || bad "package $p not found"
done

echo
if [ $PROBLEMS = 0 ]; then
  echo "ALL GOOD. Close this terminal, open a new one, and follow the run steps."
else
  echo "$PROBLEMS problem(s) above marked [FIX]. Send this output to Claude."
fi
