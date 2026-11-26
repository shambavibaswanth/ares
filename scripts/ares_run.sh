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
