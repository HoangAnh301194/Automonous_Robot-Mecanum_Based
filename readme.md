# AUTONOMOUS MECANUM ROBOT WORKFLOW (ROS 2)

## 0) Build & Source Workspace
```bash
WS=~/ros2_ws
cd ${WS}
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ${WS}/install/setup.bash
```

## 1) Launch Files for Hardware Mode

### Launch 1: LiDAR + IMU + ESP Communication
```bash
ros2 launch mo_hinh real_hw.launch.py
```
Common options:
```bash
ros2 launch mo_hinh real_hw.launch.py \
  esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

### Launch 2: Odometry (ESP or RF2O)
```bash
# Odometry from ESP
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp

# Odometry from LiDAR (RF2O)
ros2 launch mo_hinh real_odom.launch.py odom_source:=rf2o

# Use wheel encoders 1 & 4 only (exclude 2 & 3)
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp esp_wheel_odom_mode:=wheels_1_4
```
Notes:
- Two-stage launch: run Launch 2 first, then launch SLAM or Localization.
- Do not launch Launch 1 simultaneously with Launch 2 to avoid serial port conflicts.

### Launch 3: SLAM Mode (Real Robot)
```bash
# Requirement: Launch 2 must be active
ros2 launch mo_hinh real_slam.launch.py
```

### Launch 4: Localization + Nav2 Mode
```bash
# Option 1: AMCL + Nav2 (using YAML map)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=amcl \
  map_yaml:=${WS}/src/mo_hinh/maps/my_map.yaml

# Option 2: SLAM Toolbox Localization + Nav2 (using PoseGraph)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=slam_toolbox \
  map_graph:=${WS}/src/mo_hinh/maps/my_slam_graph
```

## 2) Recommended Operational Workflow
1. Start `real_odom.launch.py` to publish `/scan`, `/imu/data`, `/odom`, and TF.
2. Run SLAM: `real_slam.launch.py`.
3. Save PoseGraph map after scanning:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${WS}/src/mo_hinh/maps/my_slam_graph'}"
```
4. Save 2D Occupancy Grid map (YAML/PGM) for AMCL:
```bash
ros2 run nav2_map_server map_saver_cli -f ${WS}/src/mo_hinh/maps/my_map
```
5. Switch to Localization + Nav2 using `real_localization_nav2.launch.py` (keep `real_odom.launch.py` running).

## 3) Important Notes (TF & Goal Success)
- All real robot nodes must set `use_sim_time:=false`.
- Do not run two localization stacks simultaneously (AMCL and SLAM Toolbox Localization).
- Do not run two `odom -> base_footprint` TF sources simultaneously (ESP and RF2O).

## 4) Virtual Robot Simulation (Gazebo)
```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
ros2 launch mo_hinh virtual_slam.launch.py
```

## 5) Layer 1 Bringup & Obstacle Detector
```bash
ros2 launch layer1_bringup layer1.launch.py esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
ros2 run depth_obstacle_detector obstacle_detector --ros-args -p config_file:=${WS}/my_map/cam.yaml
```

## 6) AI Pipeline (Camera + YOLO11 Tracking + Hand Wave RTMPose)

### Step 0: Build AI Packages
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select yolo_msgs yolo_ros yolo_bringup hand_wave_detection
source ~/ros2_ws/install/setup.bash
```

### Step 1: Launch Camera Driver (Orbbec Astra Pro)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch astra_camera astra_pro.launch.xml
```
*Published topics: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/camera_info`*

### Step 2: Launch YOLO11 Detection & Tracking
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.pt \
  use_tracking:=True \
  device:=cuda:0 \
  input_image_topic:=/camera/color/image_raw
```
*Infers 2D BBoxes & Tracking IDs, published to `/yolo/tracking`.*

### Step 3: Launch Hand Wave Detection (Lightweight RTMPose with External BBoxes)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  image_topic:=/camera/color/image_raw \
  tracking_topic:=/yolo/tracking
```
> [!NOTE]
> **Lưu ý về Device (CPU / GPU) cho RTMPose**:
> - **Chạy `backend:=rtmpose`**: Dùng `device:=cpu`. RTMPose chạy thông qua ONNX Runtime. Gói `onnxruntime` mặc định từ `pip` trên Linux ARM64 (Jetson Orin) chỉ hỗ trợ CPU execution provider. Vì khung hình crop của RTMPose rất nhỏ (256x192) nên xử lý trên CPU cực kỳ nhanh (~2-5ms) và không gây tải nặng.
> - **Chạy `backend:=yolo11`**: Có thể dùng `device:=cuda:0` vì YOLO11 Pose chạy trực tiếp qua PyTorch GPU (`torch.cuda`).
> - **Để bật GPU cho RTMPose về sau**: Cần cài đặt gói `onnxruntime-gpu` (bản build riêng hỗ trợ CUDA/TensorRT cho Jetson JetPack).
*Synchronizes color images with `/yolo/tracking`, executes BatchedRTMPose, and applies temporal hand wave rules.*

### Step 4: Verify Output Topics & Visualization
```bash
# Echo per-person hand-waving status with Tracking ID:
ros2 topic echo /pose/wave_status

# Echo binary hand wave detection flag (True/False):
ros2 topic echo /pose/wave_detected

# Echo target person 3D distance & position (yolo_msgs/PersonTracking):
ros2 topic echo /person_tracking

# Echo full 3D detection bounding boxes:
ros2 topic echo /yolo/detections_3d

# Visualize debug image with skeletal keypoints and bboxes:
ros2 run rqt_image_view rqt_image_view /pose/image_debug
```

### Step 5: Run Full Automated Pipeline Script
```bash
cd ~/ros2_ws
PERSON_MODEL=yolo11n.pt HAND_WAVE_BACKEND=rtmpose bash start_person_following.sh
```

## 7) Robot GUI Web Interface
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_ui robot_ui.launch.py
```

## 8) ROS 2 Modular Backend Architecture
Hệ thống backend được chia thành các node chuyên biệt để phân chia trách nhiệm rõ ràng, quản lý từng tính năng cụ thể của robot:

- **robot_interfaces**: Chứa các định nghĩa custom messages, services, và actions (ví dụ: `RobotStatus`, `DirectGo`).
- **robot_config_manager (Phase 2)**: Quản lý việc lưu trữ, cập nhật và xóa các điểm tọa độ (locations), lộ trình (routes), và cài đặt ngôn ngữ thông qua file YAML.
- **robot_system_monitor (Phase 3)**: Thu thập, giám sát trạng thái phần cứng (pin, mức nước, trạng thái cảm biến lidar/odom) và phát tín hiệu cảnh báo kịp thời.
- **robot_navigation_manager (Phase 4)**: Xử lý các yêu cầu điều hướng của robot, giao tiếp trực tiếp với Nav2 (`NavigateToPose`) để thực hiện di chuyển chuẩn xác.
- **robot_mission_control (Phase 5)**: "Bộ não" trung tâm điều phối mọi hoạt động. Tích hợp **Behavior Tree (BT)** để quản lý linh hoạt, tự động chuyển đổi và ưu tiên các nhiệm vụ theo thứ tự:
  1. Dừng khẩn cấp (E-Stop).
  2. Về trạm sạc pin (Battery Low).
  3. Bơm nước (Water Low).
  4. Phục vụ khách hàng (Customer Request).
  5. Đi tuần tra lộ trình (Patrol).

## 9) Robot HMI GUI (Control Panel)
Giao diện điều khiển trung tâm được xây dựng bằng PyQt5 (`robot_hmi_gui.py`), kết nối và điều phối mọi hoạt động của các node Backend.

### Cách chạy HMI GUI:
```bash
cd ~/robot_ws
source install/setup.bash
python3 src/robot_hmi_gui.py
```

**Tính năng chính của HMI GUI:**
- **Map Viewer & Navigation**: Hiển thị bản đồ (từ `/map`), cập nhật các vị trí đã lưu và cho phép click trực tiếp để điều hướng robot (Nav2).
- **Cấu hình (Config)**: Thêm/Xóa nhanh các vị trí quan trọng, cài đặt ngôn ngữ và thiết lập lộ trình.
- **Giám sát (Monitor)**: Hiển thị thời gian thực thanh trạng thái pin, trạng thái nước và kết nối cảm biến.
- **Nhiệm vụ (Mission)**: Giao diện trực quan để bắt đầu/tạm ngưng tuần tra, phát tín hiệu phục vụ khách hàng, hoặc ra lệnh Dừng khẩn cấp.
