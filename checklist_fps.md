# Checklist công việc ưu tiên — Khắc phục FPS

> Nguyên tắc: **Đo → Giảm input → Giảm output → Tune pipeline → Tối ưu sâu**
> Mỗi giai đoạn kết thúc bằng benchmark. Không chuyển giai đoạn khi chưa có số liệu.

---

## Patch 0 — Instrumentation baseline ⏱️
> Mục tiêu: Có số liệu thực tế TRƯỚC KHI sửa bất kỳ thứ gì

- [x] **0.1** Thêm `time.perf_counter()` đo `predict_ms` trong `yolo_node.py` (quanh `self.yolo.predict`)
- [x] **0.2** Thêm đo `tracker_update_ms` + `image_conversion_ms` trong `tracking_node.py` (quanh `imgmsg_to_cv2` + `cvtColor` + `tracker.update`)
- [x] **0.3** Thêm đo `pose_ms` + `callback_total_ms` + `pair_age_ms` trong `ros_node.py` (quanh `self.pose()` và toàn callback)
- [x] **0.4** Log tổng hợp throttled mỗi 3 giây (không log mỗi frame)
- [ ] **0.5** Chạy trên Jetson: `sudo nvpmodel -m 0 && sudo jetson_clocks`
- [ ] **0.6** Ghi lại version: JetPack, PyTorch, ONNX Runtime, Ultralytics, Python
- [ ] **0.7** Thu thập baseline 120 giây, ghi vào bảng:

| Metric               | Giá trị baseline |
|----------------------|-----------------|
| camera_hz            |                 |
| yolo_predict_ms      |                 |
| yolo_detection_hz    |                 |
| tracking_convert_ms  |                 |
| tracker_update_ms    |                 |
| tracking_hz          |                 |
| rtmpose_ms           |                 |
| handwave_total_ms    |                 |
| handwave_hz          |                 |
| pair_age_ms (p95)    |                 |
| bbox_count_avg       |                 |
| GPU util %           |                 |
| CPU util %           |                 |
| RAM MB               |                 |
| Temperature          |                 |

### ✅ Checkpoint 0: Xác nhận thứ tự bottleneck
> Dựa trên số liệu, xác nhận: bottleneck #1 là gì? → Điều chỉnh thứ tự patch nếu cần

---

## Patch 1 — Giảm tải YOLO (rủi ro thấp, lợi ích cao) 🎯
> Mục tiêu: Giảm thời gian YOLO inference + giảm lượng output cho downstream

- [x] **1.1** Thêm parameter `classes` (string, default `''`) vào `yolo_node.py`
  - Parse `'0'` → `[0]`, `'0,2'` → `[0,2]`, `''` → `None`
  - Truyền `classes=self.classes` vào `self.yolo.predict()`
  - Chuỗi sai (e.g. `'abc'`) → `TransitionCallbackReturn.ERROR`

- [x] **1.2** Thêm `imgsz_height`, `imgsz_width`, `max_det` vào `person_follower.launch.py`
  - Bỏ hard-code `'imgsz_height': '480'` (dòng 39)
  - Default: `classes='0'`, `imgsz_height='384'`, `imgsz_width='640'`, `max_det='10'`

- [x] **1.3** Forward các parameter mới qua `yolo.launch.py`
  - `DeclareLaunchArgument` + truyền vào `yolo_node` parameters

- [x] **1.4** Cập nhật `start_person_following.sh` với biến environment:
  ```bash
  YOLO_CLASSES="${YOLO_CLASSES:-0}"
  YOLO_IMGSZ_HEIGHT="${YOLO_IMGSZ_HEIGHT:-384}"
  YOLO_MAX_DET="${YOLO_MAX_DET:-10}"
  ```

- [x] **1.5** Đổi default RTMPose device trong `start_person_following.sh` từ `cuda:0` → `cpu`
  - GPU contention giữa YOLO và RTMPose là rủi ro đã xác nhận

- [ ] **1.6** Kiểm tra letterbox behavior: chạy YOLO với `imgsz=(384,640)` trên ảnh 640×480, xác nhận resize thực tế

### ✅ Checkpoint 1: Benchmark A–B
```bash
# A: YOLO person-only, RTMPose CPU, 3D OFF, Debug OFF
# B: YOLO person-only, RTMPose GPU, 3D OFF, Debug OFF
ros2 topic hz /yolo/detections
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/wave_detected
```
> So sánh `yolo_predict_ms` với baseline. Mục tiêu giảm ≥ 20%

---

## Patch 2 — Giảm tải RTMPose + Debug (lợi ích rõ ràng) 🔧
> Mục tiêu: RTMPose chỉ chạy trên top-N người gần nhất, debug không tốn resource khi không cần

- [x] **2.1** Import `NearestTrackSelector` và `TrackedBox` vào `ros_node.py`

- [x] **2.2** Khởi tạo `self.selector = NearestTrackSelector(max_people=...)` trong `__init__`

- [x] **2.3** Refactor `sync_callback`: thay 3 list (`raw_bboxes`, `track_ids`, `confidences`) bằng `List[TrackedBox]`
  - Gọi `self.selector.select(tracked_boxes, frame.shape)` trước RTMPose
  - Chỉ đưa `selected_boxes` vào `self.pose()`

- [x] **2.4** Thêm guard: `if not selected_boxes: return` (không gọi RTMPose khi rỗng)

- [ ] **2.5** Fix `BatchedRTMPose.__call__` trong `rtmpose_batch.py`: khi `bboxes=[]` → return empty arrays thay vì chạy inference toàn frame
  ```python
  if bboxes is not None and len(bboxes) == 0:
      return np.empty((0, 17, 2), dtype=np.float32), np.empty((0, 17), dtype=np.float32)
  ```

- [x] **2.6** Thêm parameter `max_people` (ROS integer parameter, default từ config.yaml)
  - Thêm `DeclareLaunchArgument('max_people', default_value='1')` trong launch

- [x] **2.7** Conditional debug render:
  ```python
  if self.debug_pub.get_subscription_count() > 0:
      self._publish_debug_image(...)
  ```

- [ ] **2.8** (Điều kiện: Patch 0 cho thấy `tracking_convert_ms > 10ms`) Skip image conversion trong `tracking_node.py` khi dùng ByteTrack:
  - Kiểm tra Ultralytics version thực tế trên Jetson
  - Xác nhận `BYTETracker.update()` không dùng image
  - Truyền `None` thay vì ảnh đã convert

### ✅ Checkpoint 2: Benchmark C–F
```bash
# C: Production CPU pose, 3D ON, Debug OFF, max_people=1
# D: Production GPU pose, 3D ON, Debug OFF, max_people=1
# E: CPU pose, 3D ON, Debug ON, max_people=1
# F: CPU pose, 3D ON, Debug OFF, max_people=5
# G: 0 người trong cảnh (test idle)
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/wave_detected
```
> So sánh `rtmpose_ms` và `handwave_hz` với Checkpoint 1

---

## Patch 3 — Tune pipeline transport (cần dữ liệu từ Patch 0–2) 📡
> Mục tiêu: Giảm latency đầu-cuối, không tạo backlog

- [ ] **3.1** Giảm `slop` trong `tracking_node.py` từ `0.5` → `0.05`, queue từ `10` → `4`

- [ ] **3.2** Tune synchronizer `HandWaveDetection` trong `ros_node.py`:
  - `slop` từ `0.1` → `0.03`
  - `queue_size` từ `10` → `4`
  - Tính: `queue >= ceil(p95_tracking_latency / 33.3) + 1`

- [ ] **3.3** Thống nhất QoS cho image subscriber across 3 nodes:
  ```python
  image_qos = QoSProfile(
      reliability=BEST_EFFORT,
      history=KEEP_LAST,
      durability=VOLATILE,
      depth=2,
  )
  ```

- [ ] **3.4** Kiểm tra timestamp consistency:
  ```bash
  ros2 topic echo --once /camera/color/image_raw --field header.stamp
  ros2 topic echo --once /yolo/tracking --field header.stamp
  ```
  - Nếu khớp: thử `TimeSynchronizer` exact

- [ ] **3.5** Thêm metric `p99_end_to_end_latency_ms` (camera stamp → wave_detected publish)

- [ ] **3.6** Thêm `max_pair_age_ms = 250` (khởi đầu), drop pair quá cũ trong callback:
  ```python
  age_ms = (self.get_clock().now() - stamp).nanoseconds / 1e6
  if age_ms > self.max_pair_age_ms:
      self._stale_count += 1
      return
  ```

### ✅ Checkpoint 3: Soak test 10 phút
```bash
sudo tegrastats &
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/wave_detected
```
> Tiêu chí: RAM không tăng liên tục, pair_age_ms không tăng liên tục, không thermal throttle kéo dài

---

## Patch 4 — Tối ưu sâu (chỉ khi cần) 🔬
> Chỉ thực hiện nếu Patch 1–3 chưa đạt mục tiêu 15+ FPS

- [ ] **4.1** Implement latest-frame worker thread:
  - Callback chỉ lưu pair mới nhất (ghi đè cũ)
  - Worker thread lấy pair → chạy RTMPose → publish
  - Dừng worker trước khi publisher/session bị hủy

- [ ] **4.2** Thêm counter: `received_pairs`, `processed_pairs`, `overwritten_pairs`

- [ ] **4.3** Test publish từ worker thread trên ROS 2 Humble/rclpy
  - Nếu lỗi: worker tạo result → ROS timer publish

- [ ] **4.4** (Tùy chọn) Thử `MultiThreadedExecutor(num_threads=2)` với `MutuallyExclusiveCallbackGroup` cho sync callback — nhẹ hơn worker thread nhưng phải đảm bảo ONNX session thread safety

### ✅ Checkpoint 4: Regression test đầy đủ
- [ ] ByteTrack giữ ID khi người đi ngang
- [ ] Che khuất ngắn không tạo ID mới
- [ ] BOTSORT vẫn nhận image nếu chọn
- [ ] Person follower 3D hoạt động (`use_3d=True`)
- [ ] Debug image hoạt động khi mở rqt_image_view
- [ ] Hand wave detection đúng track ID
- [ ] 0 người: RTMPose không chạy, không crash
- [ ] Robot di chuyển: không false positive/negative bất thường

---

## Tiêu chí nghiệm thu cuối cùng ✅

| Tiêu chí | Ngưỡng | Cách đo |
|----------|--------|---------|
| HandWave FPS (1 người, debug OFF) | ≥ 15 Hz, mục tiêu 20–25 Hz | `ros2 topic hz /pose/wave_detected` |
| YOLO tracking FPS | ≥ 20 Hz | `ros2 topic hz /yolo/tracking` |
| End-to-end latency p95 | ≤ 150 ms | Metric nội bộ `pair_age_ms` |
| End-to-end latency p99 | ≤ 250 ms | Metric nội bộ |
| RAM sau 10 phút | Không tăng > 5% so với phút 1 | `tegrastats` |
| Track ID stability | Không đổi ID khi đi bình thường | Quan sát log |
| Thermal | Không throttle kéo dài > 30s | `tegrastats` |
| Person follower 3D | `/person_tracking` hoạt động | `ros2 topic echo` |

---

## Files liên quan

| File | Vai trò |
|------|---------|
| `src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py` | YOLO inference node |
| `src/yolo_ros/yolo_ros/yolo_ros/tracking_node.py` | ByteTrack/BOTSORT tracking |
| `src/HandWaveDetection_Pose/hand_wave_detection/ros_node.py` | RTMPose + hand wave classify |
| `src/HandWaveDetection_Pose/raised_hand/rtmpose_batch.py` | Batched RTMPose inference |
| `src/HandWaveDetection_Pose/raised_hand/logic.py` | NearestTrackSelector, RaisedHandRule |
| `src/HandWaveDetection_Pose/raised_hand/types.py` | TrackedBox, PersonPose dataclass |
| `src/HandWaveDetection_Pose/config.yaml` | max_people, confidence, pose config |
| `src/yolo_ros/yolo_bringup/launch/person_follower.launch.py` | Person follower launch |
| `src/yolo_ros/yolo_bringup/launch/yolo.launch.py` | YOLO + tracking + 3D launch |
| `start_person_following.sh` | Production startup script |
