#!/usr/bin/env bash
# ARES "recon_site" world: walled compound, road, multi-room building, lights, sky,
# and real 3D models (people, cars, hydrant, stop sign...) that YOLO can detect in Phase 4.
# Run from a ROS terminal after ares_env:  bash ~/ares_ws/scripts/ares_world_setup.sh
set -e
WS=${ARES_WS:-$HOME/ares_ws}
GZ=$WS/src/ares_gazebo
TP=$WS/third_party
W=$GZ/worlds/recon_site.world
MATFILE=/usr/share/gazebo-11/media/materials/scripts/gazebo.material
echo "=== ARES recon_site world setup ==="
[ -n "$ROS_DISTRO" ] || { echo "ERROR: run ares_env first"; exit 1; }
mkdir -p "$TP" "$GZ/models" "$GZ/worlds" "$GZ/launch"
touch "$GZ/models/.gitkeep"

# ---------- 1. Fetch only the models we need (sparse clone, kept inside the project) ----------
MODELS="person_standing person_walking hatchback suv fire_hydrant stop_sign construction_cone \
construction_barrel jersey_barrier oak_tree lamp_post dumpster cafe_table table bookshelf cardboard_box"
if [ -z "$SKIP_FETCH" ]; then
  if [ ! -d "$TP/gazebo_models/.git" ]; then
    echo "Downloading models from github.com/osrf/gazebo_models (first time only)..."
    git clone --depth 1 --filter=blob:none --sparse https://github.com/osrf/gazebo_models.git "$TP/gazebo_models"
  fi
  (cd "$TP/gazebo_models" && git sparse-checkout set $MODELS)
  for m in $MODELS; do
    if [ -d "$TP/gazebo_models/$m" ]; then
      rm -rf "$GZ/models/$m"; cp -r "$TP/gazebo_models/$m" "$GZ/models/"
      echo "  model ok:      $m"
    else
      echo "  model missing: $m (will be left out of the world)"
    fi
  done
fi
echo "[1/4] Models ready in $GZ/models"

# ---------- 2. Pick textures that exist on this machine (fallback to plain colours) ----------
mat() { grep -qE "^material $1([[:space:]]|:|$)" "$MATFILE" 2>/dev/null && echo "$1" || echo "$2"; }
GRASS=$(mat Gazebo/Grass Gazebo/Green)
BRICK=$(mat Gazebo/Bricks Gazebo/Red)
WOOD=$(mat Gazebo/WoodFloor Gazebo/Wood)
PAINT=$(mat Gazebo/PaintedWall Gazebo/White)
echo "  textures: grass=$GRASS brick=$BRICK floor=$WOOD walls=$PAINT"

# ---------- 3. Build the world ----------
wall() {  # name x y length yaw height material
  local z; z=$(awk "BEGIN{print $6/2}")
  cat >> "$W" << EOF
    <model name="$1"><static>true</static><pose>$2 $3 $z 0 0 $5</pose><link name="link">
      <collision name="c"><geometry><box><size>$4 0.2 $6</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>$4 0.2 $6</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>$7</name></script></material></visual>
    </link></model>
EOF
}
floor() {  # name x y size_x size_y z material   (visual only: robot drives on the ground plane)
  cat >> "$W" << EOF
    <model name="$1"><static>true</static><pose>$2 $3 $6 0 0 0</pose><link name="link">
      <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>$4 $5 0.002</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>$7</name></script></material></visual>
    </link></model>
EOF
}
inc() {  # model instance_name x y yaw   (skipped if the model was not downloaded)
  [ -d "$GZ/models/$1" ] || return 0
  cat >> "$W" << EOF
    <include><uri>model://$1</uri><name>$2</name><static>true</static><pose>$3 $4 0 0 0 $5</pose></include>
EOF
}
light() {  # name x y "r g b"
  cat >> "$W" << EOF
    <light name="$1" type="point"><pose>$2 $3 1.3 0 0 0</pose><diffuse>$4 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular><cast_shadows>false</cast_shadows>
      <attenuation><range>6</range><constant>0.4</constant><linear>0.15</linear><quadratic>0.02</quadratic></attenuation></light>
EOF
}

cat > "$W" << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="recon_site">
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <shadows>true</shadows>
      <sky><time>15</time><clouds><speed>3</speed><humidity>0.4</humidity></clouds></sky>
    </scene>
    <include><uri>model://ground_plane</uri></include>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.88 0.8 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
    </light>
    <gui fullscreen="0"><camera name="user_camera"><pose>-13 -15 15 0 0.62 0.82</pose></camera></gui>
EOF

# ground surfaces
floor grass    1    0   22 14  0.001 "$GRASS"
floor road     1    5   22  2  0.003 Gazebo/DarkGrey
i=0; for x in -9 -6 -3 0 3 6 9; do i=$((i+1)); floor "lane_$i" $x 5 1.2 0.12 0.005 Gazebo/White; done
floor bldg_floor 7  -2   8  8  0.003 "$WOOD"
floor path     1.5 -2.4  3  1.2 0.004 Gazebo/Grey

# compound perimeter (22 m x 14 m, low concrete wall)
wall perim_n  1    7  22.2 0      0.8 Gazebo/Grey
wall perim_s  1   -7  22.2 0      0.8 Gazebo/Grey
wall perim_w -10   0  14.2 1.5708 0.8 Gazebo/Grey
wall perim_e  12   0  14.2 1.5708 0.8 Gazebo/Grey

# building: 8 m x 8 m, brick outside, painted inside, three rooms with doorways
wall b_south  7  -6    8.2 0      1.4 "$BRICK"
wall b_north  7   2    8.2 0      1.4 "$BRICK"
wall b_east  11  -2    8.2 1.5708 1.4 "$BRICK"
wall b_west1  3  -4.5  3.0 1.5708 1.4 "$BRICK"   # entrance gap y -3.0 .. -1.8
wall b_west2  3   0.1  3.8 1.5708 1.4 "$BRICK"
wall in_1     7  -3.6  4.8 1.5708 1.4 "$PAINT"   # doorway y -1.2 .. 0
wall in_2     7   1.0  2.0 1.5708 1.4 "$PAINT"
wall in_3     8.4 -2   2.8 0      1.4 "$PAINT"   # doorway x 9.8 .. 11

# indoor lights (warm lobby and office, red "emergency" light in the back room)
light light_lobby   5  -2  "1 0.85 0.6"
light light_office  9   0  "0.9 0.9 1"
light light_alert   9  -4  "1 0.2 0.2"

# outdoor objects
inc hatchback           car_1      -6    5     0
inc suv                 car_2       7    5     3.1416
inc fire_hydrant        hydrant    -1.5  3.2   0
inc stop_sign           stop_sign   2.2  3.4   1.5708
inc lamp_post           lamp_1     -3    3.4   0
inc lamp_post           lamp_2      5    3.4   0
inc construction_cone   cone_1     -4   -1     0
inc construction_cone   cone_2     -4   -2     0
inc construction_cone   cone_3     -4   -3     0
inc construction_cone   cone_4     -3.4 -4     0
inc construction_barrel barrel_1   -7.5 -4     0
inc construction_barrel barrel_2   -8.2 -4.6   0
inc jersey_barrier      barrier_1  -6.5 -1     1.5708
inc dumpster            dumpster_1 -8.6  0.8   1.5708
inc oak_tree            tree_1     -8.8 -6     0
inc oak_tree            tree_2     -8.8  2.6   0
inc person_walking      person_out -1   -5.2   0.8

# indoor objects (a "person to find" in the back room)
inc cafe_table          table_lobby   5   -0.5  0
inc bookshelf           shelf_lobby   4.5  1.5  0
inc cardboard_box       box_1         6.3 -5.3  0
inc cardboard_box       box_2         5.8 -5.4  0.4
inc table               table_office  9    0.8  0
inc bookshelf           shelf_office 10.3  1.5  0
inc cardboard_box       box_3         8   -5.3  0.2
inc person_standing     person_in     9.5 -4.5  2.3

cat >> "$W" << 'EOF'
  </world>
</sdf>
EOF
echo "[2/4] World written: $W"

# ---------- 4. Launch file, install rules, gitignore ----------
cat > "$GZ/launch/recon.launch.py" << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz = get_package_share_directory('ares_gazebo')
    model_path = os.path.join(gz, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),   # never wait on the internet
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'sim.launch.py')),
            launch_arguments={'world': os.path.join(gz, 'worlds', 'recon_site.world')}.items()),
    ])
EOF
CM=$GZ/CMakeLists.txt
grep -q "install(DIRECTORY worlds launch models" "$CM" || \
  sed -i 's|install(DIRECTORY worlds launch DESTINATION|install(DIRECTORY worlds launch models DESTINATION|' "$CM"
GI=$WS/.gitignore
for line in 'third_party/' 'src/ares_gazebo/models/*' '!src/ares_gazebo/models/.gitkeep'; do
  grep -qxF "$line" "$GI" 2>/dev/null || echo "$line" >> "$GI"
done
echo "[3/4] recon.launch.py, install rules and .gitignore updated"

if [ -z "$SKIP_BUILD" ]; then
  command -v gz > /dev/null && { gz sdf -k "$W" || echo "WARNING: world check failed"; }
  cd "$WS" && colcon build --symlink-install --packages-select ares_gazebo
fi
echo "[4/4] Done. Launch with:  ros2 launch ares_gazebo recon.launch.py"
