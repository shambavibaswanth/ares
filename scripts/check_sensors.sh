#!/usr/bin/env bash
echo "--- topics ---"
ros2 topic list | grep -E "scan|camera|imu|odom|ultrasonic|cmd_vel" || echo "no ARES topics (is the sim running?)"
for t in /scan /camera/image_raw /imu /odom; do
  echo "--- $t (expect: scan~10, camera~30, imu~100, odom~50 Hz) ---"
  timeout 6 ros2 topic hz $t 2>/dev/null | grep "average rate" | tail -1 || echo "NO DATA on $t"
done
for s in front left right; do
  echo "--- /ultrasonic/$s range ---"
  timeout 5 ros2 topic echo /ultrasonic/$s --once --field range 2>/dev/null || echo "NO DATA on /ultrasonic/$s"
done
