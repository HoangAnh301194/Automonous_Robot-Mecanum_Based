# MO HINH - REAL ROBOT WORKFLOW (ROS 2)

## 0) Build + source
```bash
WS=~/ros2_ws
cd ${WS}
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ${WS}/install/setup.bash
```

## 1) 4 launch file cho che do thuc

### File 1: Launch LiDAR + IMU + ESP communication
```bash
ros2 launch mo_hinh real_hw.launch.py
```
Tuy chon thuong dung:
```bash
ros2 launch mo_hinh real_hw.launch.py \
  esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

### File 2: Launch odom, chon nguon ESP hoac RF2O
```bash
# odom tu ESP
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp

# odom tu lidar RF2O
ros2 launch mo_hinh real_odom.launch.py odom_source:=rf2o

# chi dung encoder banh 1 va 4 (bo 2 va 3)
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp esp_wheel_odom_mode:=wheels_1_4
```
Ghi chu:
- Kieu 2 tang: chay `file2` truoc, sau do moi chay `file3` hoac `file4`.
- Khi da chay `file2`, khong chay them `file1` de tranh trung node serial.

### File 3: Launch che do SLAM (robot that)
```bash
# Yeu cau: file2 dang chay san
ros2 launch mo_hinh real_slam.launch.py
```

### File 4: Launch che do dinh vi + Nav2
```bash
# Lua chon 1: AMCL + Nav2 (dung map yaml)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=amcl \
  map_yaml:=${WS}/src/mo_hinh/maps/my_map.yaml

# Lua chon 2: slam_toolbox localization + Nav2 (dung posegraph)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=slam_toolbox \
  map_graph:=${WS}/src/mo_hinh/maps/my_slam_graph
```

## 2) Luong van hanh khuyen nghi
1. Chay `file2` (`real_odom.launch.py`) de co `/scan`, `/imu/data`, `/odom`, TF.
2. Chay SLAM: `real_slam.launch.py`.
3. Luu map posegraph sau khi quet xong:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: '${WS}/src/mo_hinh/maps/my_slam_graph'}"
```
4. Tao map yaml/pgm de AMCL dung:
```bash
ros2 run nav2_map_server map_saver_cli -f ${WS}/src/mo_hinh/maps/my_map
```
5. Chuyen sang dinh vi + Nav2 bang `real_localization_nav2.launch.py` (giu `file2` dang chay).

## 3) Luu y quan trong de tranh loi TF/goal success gia
- Tat ca node robot that phai `use_sim_time:=false`.
- Khong chay dong thoi 2 stack localization (AMCL va slam_toolbox localization).
- Khong chay dong thoi 2 nguon TF `odom -> base_footprint` (ESP va RF2O).

## 4) Che do robot ao (giu nguyen)
```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
ros2 launch mo_hinh virtual_slam.launch.py
```
ros2 run depth_obstacle_detector obstacle_detector --ros-args -p config_file:=/home/orin/ros2_ws/my_map/nen_final.yaml

