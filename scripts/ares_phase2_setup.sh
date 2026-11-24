#!/usr/bin/env bash
# ARES Phase 2 setup: robot model, Gazebo world, launch file, helper scripts, build.
set -e
WS=~/ares_ws
SRC=$WS/src
echo "=== ARES Phase 2 setup ==="

PY=$(which python3)
echo "python3 -> $PY"
if [ "$PY" != "/usr/bin/python3" ]; then
  echo "ERROR: python3 is not /usr/bin/python3 (OMNeT++ venv still active). Run ares_env first."; exit 1
fi
if [ -z "$ROS_DISTRO" ]; then echo "ERROR: ROS 2 not sourced. Run ares_env first."; exit 1; fi
for d in ares_description ares_gazebo; do
  [ -d "$SRC/$d" ] || { echo "ERROR: $SRC/$d missing"; exit 1; }
done
mkdir -p $SRC/ares_description/{urdf,meshes,rviz} $SRC/ares_gazebo/{worlds,launch} $WS/scripts $WS/results

# ---------- 1. Robot model ----------
cat > $SRC/ares_description/urdf/ares.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ares">
  <xacro:property name="PI" value="3.14159265"/>
  <xacro:property name="base_l" value="0.40"/>
  <xacro:property name="base_w" value="0.28"/>
  <xacro:property name="base_h" value="0.10"/>
  <xacro:property name="base_m" value="6.0"/>
  <xacro:property name="wheel_r" value="0.05"/>
  <xacro:property name="wheel_w" value="0.04"/>
  <xacro:property name="wheel_m" value="0.5"/>
  <xacro:property name="wheel_sep" value="0.32"/>

  <material name="body"><color rgba="0.15 0.35 0.2 1"/></material>
  <material name="dark"><color rgba="0.1 0.1 0.1 1"/></material>
  <material name="sensor"><color rgba="0.8 0.2 0.1 1"/></material>

  <link name="base_footprint"/>
  <link name="base_link">
    <visual><geometry><box size="${base_l} ${base_w} ${base_h}"/></geometry><material name="body"/></visual>
    <collision><geometry><box size="${base_l} ${base_w} ${base_h}"/></geometry></collision>
    <inertial>
      <mass value="${base_m}"/>
      <inertia ixx="${base_m/12*(base_w*base_w+base_h*base_h)}" ixy="0" ixz="0"
               iyy="${base_m/12*(base_l*base_l+base_h*base_h)}" iyz="0"
               izz="${base_m/12*(base_l*base_l+base_w*base_w)}"/>
    </inertial>
  </link>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
  </joint>
  <gazebo reference="base_link"><material>Gazebo/Green</material></gazebo>

  <xacro:macro name="wheel" params="name y">
    <link name="${name}_wheel">
      <visual><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry><material name="dark"/></visual>
      <collision><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry></collision>
      <inertial>
        <mass value="${wheel_m}"/>
        <inertia ixx="${wheel_m/12*(3*wheel_r*wheel_r+wheel_w*wheel_w)}" ixy="0" ixz="0"
                 iyy="${wheel_m/12*(3*wheel_r*wheel_r+wheel_w*wheel_w)}" iyz="0"
                 izz="${wheel_m*wheel_r*wheel_r/2}"/>
      </inertial>
    </link>
    <joint name="${name}_wheel_joint" type="continuous">
      <parent link="base_link"/><child link="${name}_wheel"/>
      <origin xyz="0 ${y} -0.01" rpy="${-PI/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
    <gazebo reference="${name}_wheel">
      <mu1>1.0</mu1><mu2>1.0</mu2><kp>500000.0</kp><kd>10.0</kd>
      <material>Gazebo/Black</material>
    </gazebo>
  </xacro:macro>
  <xacro:wheel name="left"  y="${wheel_sep/2}"/>
  <xacro:wheel name="right" y="${-wheel_sep/2}"/>

  <xacro:macro name="caster" params="name x">
    <link name="${name}_caster">
      <visual><geometry><sphere radius="0.02"/></geometry><material name="dark"/></visual>
      <collision><geometry><sphere radius="0.02"/></geometry></collision>
      <inertial><mass value="0.05"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
    </link>
    <joint name="${name}_caster_joint" type="fixed">
      <parent link="base_link"/><child link="${name}_caster"/>
      <origin xyz="${x} 0 -0.04" rpy="0 0 0"/>
    </joint>
    <gazebo reference="${name}_caster"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  </xacro:macro>
  <xacro:caster name="front" x="0.15"/>
  <xacro:caster name="rear"  x="-0.15"/>

  <link name="lidar_link">
    <visual><geometry><cylinder radius="0.04" length="0.05"/></geometry><material name="sensor"/></visual>
    <inertial><mass value="0.1"/><inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/></inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/><child link="lidar_link"/>
    <origin xyz="0.05 0 0.075" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual><geometry><box size="0.03 0.08 0.03"/></geometry><material name="sensor"/></visual>
    <inertial><mass value="0.05"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="0.2 0 0.04" rpy="0 0 0"/>
  </joint>
  <link name="camera_optical"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/><child link="camera_optical"/>
    <origin xyz="0 0 0" rpy="${-PI/2} 0 ${-PI/2}"/>
  </joint>

  <link name="imu_link">
    <inertial><mass value="0.01"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>

  <xacro:macro name="ultrasonic" params="name x y yaw">
    <link name="ultra_${name}_link">
      <visual><geometry><box size="0.02 0.04 0.02"/></geometry><material name="sensor"/></visual>
      <inertial><mass value="0.01"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
    </link>
    <joint name="ultra_${name}_joint" type="fixed">
      <parent link="base_link"/><child link="ultra_${name}_link"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 ${yaw}"/>
    </joint>
    <gazebo reference="ultra_${name}_link">
      <sensor name="ultra_${name}" type="ray">
        <always_on>true</always_on><visualize>false</visualize><update_rate>10</update_rate>
        <ray>
          <scan><horizontal>
            <samples>5</samples><resolution>1</resolution>
            <min_angle>-0.15</min_angle><max_angle>0.15</max_angle>
          </horizontal></scan>
          <range><min>0.02</min><max>4.0</max><resolution>0.01</resolution></range>
        </ray>
        <plugin name="ultra_${name}_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros><remapping>~/out:=ultrasonic/${name}</remapping></ros>
          <output_type>sensor_msgs/Range</output_type>
          <radiation_type>ultrasound</radiation_type>
          <frame_name>ultra_${name}_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>
  <xacro:ultrasonic name="front" x="0.2"  y="0"     yaw="0"/>
  <xacro:ultrasonic name="left"  x="0.0"  y="0.14"  yaw="${PI/2}"/>
  <xacro:ultrasonic name="right" x="0.0"  y="-0.14" yaw="${-PI/2}"/>

  <gazebo reference="lidar_link">
    <material>Gazebo/Red</material>
    <sensor name="lidar" type="ray">
      <always_on>true</always_on><visualize>false</visualize><update_rate>10</update_rate>
      <ray>
        <scan><horizontal>
          <samples>360</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
        </horizontal></scan>
        <range><min>0.12</min><max>10.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
    <sensor name="camera" type="camera">
      <always_on>true</always_on><update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.3963</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>20</far></clip>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <camera_name>camera</camera_name>
        <frame_name>camera_optical</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on><update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros><remapping>~/out:=imu</remapping></ros>
        <frame_name>imu_link</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${wheel_sep}</wheel_separation>
      <wheel_diameter>${2*wheel_r}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>2.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>
</robot>
EOF
echo "[1/6] URDF written"

# ---------- 2. World ----------
cat > $SRC/ares_gazebo/worlds/obstacle_room.world << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="obstacle_room">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <model name="walls">
      <static>true</static>
      <link name="link">
        <collision name="n_c"><pose>0 5 0.5 0 0 0</pose><geometry><box><size>10.2 0.2 1</size></box></geometry></collision>
        <visual name="n_v"><pose>0 5 0.5 0 0 0</pose><geometry><box><size>10.2 0.2 1</size></box></geometry></visual>
        <collision name="s_c"><pose>0 -5 0.5 0 0 0</pose><geometry><box><size>10.2 0.2 1</size></box></geometry></collision>
        <visual name="s_v"><pose>0 -5 0.5 0 0 0</pose><geometry><box><size>10.2 0.2 1</size></box></geometry></visual>
        <collision name="e_c"><pose>5 0 0.5 0 0 0</pose><geometry><box><size>0.2 10.2 1</size></box></geometry></collision>
        <visual name="e_v"><pose>5 0 0.5 0 0 0</pose><geometry><box><size>0.2 10.2 1</size></box></geometry></visual>
        <collision name="w_c"><pose>-5 0 0.5 0 0 0</pose><geometry><box><size>0.2 10.2 1</size></box></geometry></collision>
        <visual name="w_v"><pose>-5 0 0.5 0 0 0</pose><geometry><box><size>0.2 10.2 1</size></box></geometry></visual>
      </link>
    </model>
    <model name="box1"><static>true</static><pose>2.5 1.5 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.8 0.8 0.8</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual>
      </link></model>
    <model name="box2"><static>true</static><pose>-2.0 -2.5 0.5 0 0 0.5</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>1.2 0.6 1.0</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>1.2 0.6 1.0</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material></visual>
      </link></model>
    <model name="pillar"><static>true</static><pose>-1.5 2.5 0.6 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.3</radius><length>1.2</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.3</radius><length>1.2</length></cylinder></geometry>
          <material><ambient>0.9 0.8 0.1 1</ambient><diffuse>0.9 0.8 0.1 1</diffuse></material></visual>
      </link></model>
    <model name="wall_partition"><static>true</static><pose>1.0 -2.0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>3.0 0.2 1.0</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>3.0 0.2 1.0</size></box></geometry></visual>
      </link></model>
  </world>
</sdf>
EOF
echo "[2/6] World written"

# ---------- 3. Launch file ----------
cat > $SRC/ares_gazebo/launch/sim.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_dir = get_package_share_directory('ares_description')
    gz_dir = get_package_share_directory('ares_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(gz_dir, 'worlds', 'obstacle_room.world')
    xacro_file = os.path.join(desc_dir, 'urdf', 'ares.urdf.xacro')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui, 'verbose': 'true'}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ares',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-timeout', '120'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, rsp, spawn,
    ])
EOF
echo "[3/6] Launch file written"

# ---------- 4. Package dependencies ----------
add_dep() {
  grep -q "<exec_depend>$2</exec_depend>" "$1" || \
    sed -i "s|<buildtool_depend>ament_cmake</buildtool_depend>|<buildtool_depend>ament_cmake</buildtool_depend>\n  <exec_depend>$2</exec_depend>|" "$1"
}
for dep in xacro robot_state_publisher; do add_dep $SRC/ares_description/package.xml $dep; done
for dep in xacro robot_state_publisher gazebo_ros gazebo_plugins ares_description; do
  add_dep $SRC/ares_gazebo/package.xml $dep
done
grep -q "install(DIRECTORY urdf" $SRC/ares_description/CMakeLists.txt || \
  sed -i 's|^ament_package()|install(DIRECTORY urdf meshes rviz DESTINATION share/${PROJECT_NAME})\n\nament_package()|' $SRC/ares_description/CMakeLists.txt
grep -q "install(DIRECTORY worlds" $SRC/ares_gazebo/CMakeLists.txt || \
  sed -i 's|^ament_package()|install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})\n\nament_package()|' $SRC/ares_gazebo/CMakeLists.txt
touch $SRC/ares_description/meshes/.gitkeep $SRC/ares_description/rviz/.gitkeep
echo "[4/6] Package dependencies set"

# ---------- 5. Helper scripts ----------
cat > $WS/scripts/kill_gazebo.sh << 'EOF'
#!/usr/bin/env bash
pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null
sleep 2
if pgrep -af "gzserver|gzclient" > /dev/null; then echo "Still running:"; pgrep -af "gzserver|gzclient"; else echo "clean"; fi
EOF

cat > $WS/scripts/check_sensors.sh << 'EOF'
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
EOF
chmod +x $WS/scripts/kill_gazebo.sh $WS/scripts/check_sensors.sh
echo "[5/6] Helper scripts written"

# ---------- 6. Validate and build ----------
cd $WS
xacro $SRC/ares_description/urdf/ares.urdf.xacro > /tmp/ares.urdf && echo "xacro OK"
if command -v gz > /dev/null; then gz sdf -k $SRC/ares_gazebo/worlds/obstacle_room.world || echo "WARNING: world check failed"; fi
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "[6/6] Build done"
