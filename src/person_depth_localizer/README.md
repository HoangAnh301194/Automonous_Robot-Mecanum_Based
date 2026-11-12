# Person Depth Localizer (Multi-Person 3D Preview)

Package định vị người 3D thô (Uncalibrated Multi-person 3D Preview) sử dụng luồng **RGB 640×480** và **Depth 160×120** từ Astra Pro AI, kết hợp **YOLO Tracking** và **TF2**.

> ⚠️ **LƯU Ý:** Đây là bản prototype phục vụ kiểm tra dữ liệu và hiển thị trực quan trên RViz. **KHÔNG** dùng làm mục tiêu điều hướng tự động (Nav2 Goal) cho tới khi hoàn thành hiệu chuẩn camera (Calibration).

---

## 1. Các Topic Chính (Topics Published)

| Topic | Định dạng Message | Ý nghĩa |
| :--- | :--- | :--- |
| `/perception/tracked_people` | `robot_interfaces/msg/TrackedPersonArray` | **Topic chính**: Mảng chứa tất cả người được track (tọa độ $x, y, z$, khoảng cách $d_{xy}$, depth, track_id, timestamp) |
| `/mission/person_marker_array` | `visualization_msgs/msg/MarkerArray` | Cột trụ 3D (Cylinder) và nhãn chữ (Text `ID:... \| d:...m`) hiển thị trên RViz |
| `/mission/person_preview_status` | `std_msgs/msg/String` | JSON log trạng thái (`preview_only`, số người phát hiện, độ trễ skew, v.v.) |
| `/mission/person_preview_base` | `geometry_msgs/msg/PointStamped` | Tọa độ 2D mặt sàn của người chính (bbox to nhất) trên `base_footprint` |
| `/mission/person_distance_preview` | `std_msgs/msg/Float32` | Khoảng cách mặt phẳng (m) của người chính (phục vụ debug nhanh / rqt_plot) |

---

## 2. Hướng Dẫn Biên Dịch (Build)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to person_depth_localizer mo_hinh yolo_bringup
source install/setup.bash
```

---

## 3. Các Lệnh Khởi Chạy (Step-by-Step Launch)

### Terminal 1: Khởi động 3 Camera (Depth 160×120 + RGB AI 640×480)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch mo_hinh three_astra.launch.py
```

### Terminal 2: Khởi động YOLO Tracking (TensorRT Engine trên CUDA)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/orin/ros2_ws/yolo11n.engine \
  device:=cuda:0 \
  input_image_topic:=/ai_camera/color/image_raw \
  image_reliability:=2 \
  imgsz_height:=384 \
  imgsz_width:=640 \
  classes:=0 \
  use_tracking:=True \
  use_3d:=False
```

### Terminal 3: Khởi động Node Định Vị Người Đa Đối Tượng
*(Khi `robot_state_publisher` đang phát TF từ `src/mo_hinh/urdf/xe.urdf`, **không** bật `publish_test_tf`, vì sẽ tạo publisher trùng cho cùng frame camera. Camera AI cao 1,20 m so với mặt sàn, LiDAR cao 0,30 m trong URDF.)*
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch person_depth_localizer rough_person_preview.launch.py \
  publish_test_tf:=false \
  depth_frame:=ai_camera_depth_optical_frame
```

`publish_test_tf:=true` chỉ dùng trong thí nghiệm **độc lập không chạy robot_state_publisher**, với `test_z:=1.2`. Không chạy đồng thời hai nguồn TF cho cùng child frame.

### Terminal 4 (Tùy chọn): Khởi động Nhận Diện Giơ Tay (RTMPose)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  tracking_topic:=/yolo/tracking \
  image_topic:=/ai_camera/color/image_raw \
  max_people:=3
```

---

## 4. Lệnh Kiểm Tra Dữ Liệu Thực Tế (Verification)

```bash
source /opt/ros/humble/setup.bash

# 1. Xem mảng tất cả người đang được theo dõi:
ros2 topic echo /perception/tracked_people

# 2. Xem trạng thái kết nối & số lượng người:
ros2 topic echo /mission/person_preview_status

# 3. Xem khoảng cách người gần nhất (m):
ros2 topic echo /mission/person_distance_preview

# 4. Kiểm tra nhận diện giơ tay (True/False):
ros2 topic echo /pose/wave_detected
```

### Cấu hình xem trên RViz2:
1. Mở `rviz2`.
2. Đặt **Fixed Frame** = `base_footprint` (hoặc `map` nếu đã chạy SLAM/AMCL).
3. Bấm **Add** -> chọn **By topic** -> `/mission/person_marker_array`.
4. Mỗi người sẽ được đại diện bằng một cột trụ màu cam cùng nhãn Track ID và khoảng cách nổi trên đầu.

---

## 5. Lệnh Dừng / Kill Toàn Bộ (Clean Teardown)

Để dừng sạch sẽ tất cả các camera, node YOLO, AI và static transform đang chạy nền chỉ bằng 1 dòng lệnh:

```bash
killall -9 astra_camera_node usb_cam_node_exe yolo_node tracking_node debug_node rough_localizer hand_wave_detector static_transform_publisher 2>/dev/null; ros2 daemon stop
```
