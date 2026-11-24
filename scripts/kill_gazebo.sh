#!/usr/bin/env bash
pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null
sleep 2
if pgrep -af "gzserver|gzclient" > /dev/null; then echo "Still running:"; pgrep -af "gzserver|gzclient"; else echo "clean"; fi
