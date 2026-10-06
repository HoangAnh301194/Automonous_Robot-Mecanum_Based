# Kế hoạch khắc phục FPS cho YOLO ROS + HandWaveDetection

## 1. Mục tiêu

Tài liệu giải thích nguyên nhân FPS của `HandWaveDetection` giảm sau khi bỏ YOLO
detector nội bộ và chuyển sang nhận bounding box từ `/yolo/tracking`. Tài liệu
cũng mô tả implementation, ràng buộc, benchmark, nghiệm thu và rollback.

Mục tiêu kỹ thuật:

- Không chạy YOLO detector thứ hai trong `HandWaveDetection`.
- Tái sử dụng detection và track ID từ `yolo_ros`.
- Không xử lý backlog frame cũ khi hệ thống quá tải.
- Giới hạn số ROI đưa vào RTMPose.
- Giữ temporal hand-wave filter ổn định theo ByteTrack ID.
- Không làm hỏng person follower 3D.
- Tối thiểu 15 FPS; mục tiêu 20–25 FPS với một người.
- Đo riêng YOLO, tracking, RTMPose, debug và latency đầu-cuối.

Phạm vi tài liệu: nhánh `main`, commit `650469d`.

## 2. Bằng chứng từ Git

```text
e12d3de  Pipeline cũ: HandWaveDetection tự chạy YOLO + RTMPose
   |
43d0585  2026-08-11: bỏ YOLO nội bộ, subscribe /yolo/tracking
   |
650469d  2026-08-11: sửa cách đọc yolo_msgs
```

Commit đổi kiến trúc chính là `43d0585`, không phải merge `18aef9c`.

```bash
git log --graph --decorate --oneline --all
git show --stat 43d0585
git diff 43d0585^ 43d0585 -- \
  src/HandWaveDetection_Pose/hand_wave_detection/ros_node.py
git show 43d0585^:src/HandWaveDetection_Pose/hand_wave_detection/ros_node.py
```

## 3. Kiến trúc trước và sau

### 3.1. Pipeline cũ

```text
/camera/color/image_raw
        |
        v
HandWaveDetection process
        +-- YOLO11.track(classes=[0], imgsz=640x384)
        |     +-- ByteTrack nội bộ
        |     +-- lọc person
        |     +-- chọn tối đa max_people
        +-- RTMPose batch
        +-- raised-hand rule
        +-- temporal filter
        +-- debug image
```

Đặc điểm:

- Một callback nhận ảnh.
- YOLO và RTMPose chạy nối tiếp trên cùng frame.
- YOLO chỉ detect person, input `640 x 384`.
- `NearestTrackSelector` áp dụng `max_people`.
- Không ROS serialization giữa detector và pose estimator.
- Không synchronizer thứ hai cho HandWaveDetection.

### 3.2. Pipeline hiện tại

```text
/camera/color/image_raw
        +--> yolo_node
        |      +-- YOLO11.predict(640x480)
        |      +-- mọi class, max_det=300
        |      +-- Results GPU -> CPU
        |             |
        |             v
        |        /yolo/detections
        |
        +--> tracking_node
        |      +-- ApproximateTimeSynchronizer
        |      +-- cv_bridge copy, BGR -> RGB
        |      +-- ByteTrack
        |             |
        |             v
        |        /yolo/tracking
        |
        +--> HandWaveDetection
               +-- ApproximateTimeSynchronizer
               +-- cv_bridge copy
               +-- RTMPose cho mọi person bbox
               +-- luôn tạo debug image
```

Pipeline 3D chạy song song:

```text
/yolo/tracking + depth + camera_info
        |
        v
detect_3d_node -> /yolo/detections_3d -> follower_node
```

Đường 3D cạnh tranh CPU, memory bandwidth, DDS bandwidth và executor time.

## 4. Vì sao bỏ YOLO nội bộ nhưng FPS lại giảm

### 4.1. `/yolo/tracking` trở thành rate limiter

```text
FPS HandWaveDetection <= FPS /yolo/tracking <= FPS /yolo/detections
```

RTMPose không thể chạy 25 FPS nếu `/yolo/tracking` chỉ publish 12 FPS. Bỏ YOLO
nội bộ loại bỏ một model nhưng thêm ROS transport, tracking process và đồng bộ
timestamp.

### 4.2. YOLO mới nặng hơn cấu hình cũ

YOLO cũ trong `RTMPoseBackend`:

```python
self.detector.track(
    frame,
    classes=[0],
    imgsz=(384, 640),
    ...
)
```

YOLO hiện tại trong `yolo_ros`:

```python
self.yolo.predict(
    source=cv_image,
    imgsz=(480, 640),
    max_det=300,
    ...
)
```

Khác biệt:

- `480 x 640` có số pixel đầu vào lớn hơn `384 x 640` 25%.
- Không truyền `classes=[0]`; model giữ kết quả mọi class COCO.
- `max_det=300` quá lớn cho person following.
- NMS, message parsing, tracking và serialization xử lý nhiều output hơn.

`classes=[0]` chủ yếu giảm hậu xử lý và số output; không loại bỏ toàn bộ chi phí
backbone YOLO.

### 4.3. Hai tầng đồng bộ timestamp

`tracking_node` dùng:

```python
ApproximateTimeSynchronizer(..., queue_size=10, slop=0.5)
```

`HandWaveDetection` dùng:

```python
ApproximateTimeSynchronizer(..., queue_size=10, slop=0.1)
```

Tác động:

- Hai hàng đợi giữ ảnh và detection cùng lúc.
- Khi inference chậm hơn camera, frame cũ có thể tiếp tục được xử lý.
- FPS thấp đồng thời latency có thể tăng dần.
- `slop=0.5` quá rộng với camera 30 FPS.
- Queue 1 cũng có thể sai: detection đến sau ảnh; ảnh cùng timestamp phải còn
  trong cache khi detection tới.

Mục tiêu không phải queue nhỏ nhất. Mục tiêu là đủ giữ ảnh trong thời gian YOLO
latency nhưng không tạo backlog dài.

### 4.4. Ảnh bị copy và chuyển màu nhiều lần

Một frame màu hiện được xử lý tối thiểu tại:

1. `yolo_node`: `imgmsg_to_cv2`.
2. `tracking_node`: `imgmsg_to_cv2` và `cv2.cvtColor`.
3. `HandWaveDetection`: `imgmsg_to_cv2`.
4. `HandWaveDetection`: vẽ debug và `cv2_to_imgmsg`.

Với Python process tách rời, ROS 2 không tự đảm bảo zero-copy. Trên Jetson,
memory bandwidth và CPU copy có thể thành bottleneck trước GPU.

### 4.5. Mất giới hạn `max_people`

Pipeline cũ:

```python
selected = self.selector.select(tracked_boxes, frame.shape)
```

Pipeline mới đưa toàn bộ bbox vào RTMPose:

```python
keypoints_arr, scores_arr = self.pose(frame, bboxes=expanded_bboxes)
```

`processing.max_people: 5` vẫn có trong `config.yaml` nhưng không được dùng trong
ROS node mới. Cảnh nhiều người hoặc false positive làm latency RTMPose tăng.

### 4.6. Debug image luôn được tạo

Ngay cả khi không có subscriber `/pose/image_debug`, node vẫn vẽ bbox/skeleton,
render text, chuyển OpenCV image thành ROS `Image`, rồi publish.

### 4.7. GPU contention

Nếu cả hai dùng GPU:

```text
yolo_node       -> PyTorch CUDA context
hand_wave_node  -> ONNX Runtime CUDA context
```

Hai process cạnh tranh CUDA compute, GPU memory bandwidth, workspace memory và
context scheduling. RTMPose `256 x 192` nhỏ; RTMPose CPU đôi khi cho throughput
toàn hệ thống tốt hơn do không tranh GPU với YOLO. Phải benchmark đầu-cuối.

### 4.8. Pipeline 3D chạy đồng thời

`use_3d=True` làm `detect_3d_node` đồng bộ depth, camera info, tracking; xử lý
depth ROI; tra TF; publish detection 3D. Không tắt 3D trong production nếu
`follower_node` cần distance. Có thể tắt khi benchmark riêng HandWaveDetection.

## 5. Mục tiêu kiến trúc sau sửa

```text
/camera/color/image_raw
        +--> yolo_node
        |      - person only, 640x384, max_det nhỏ
        +--> tracking_node
        |      - queue hữu hạn, giữ ID ổn định
        +--> hand_wave_detection
               - ghép đúng timestamp
               - chọn top max_people
               - RTMPose batch
               - drop frame cũ khi quá tải
               - chỉ render debug khi cần
```

Thứ tự ưu tiên:

1. Giảm công việc trước RTMPose.
2. Khôi phục giới hạn ROI.
3. Bỏ chi phí debug không cần thiết.
4. Đo latency và FPS từng tầng.
5. Sau cùng mới đổi threading hoặc tracker internals.

## 6. Implementation giai đoạn 1 — Patch ít rủi ro

### 6.1. Thêm lọc class vào `yolo_node`

File `src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py`.

Không đặt mặc định `[0]` trong package generic. Chuỗi rỗng giữ detect mọi class;
launch person follower override thành `0`.

Khai báo:

```python
self.declare_parameter('classes', '')
```

Đọc và kiểm tra trong `on_configure`:

```python
classes_value = (
    self.get_parameter('classes')
    .get_parameter_value()
    .string_value
    .strip()
)
try:
    self.classes = (
        [int(item.strip()) for item in classes_value.split(',')]
        if classes_value
        else None
    )
except ValueError:
    self.get_logger().error(
        f'Invalid classes parameter: {classes_value}'
    )
    return TransitionCallbackReturn.ERROR
```

Truyền `classes=self.classes` vào `self.yolo.predict(...)`.

Ràng buộc:

- COCO person ID là `0` với `yolo11n.pt` chuẩn.
- Model custom có thể dùng ID khác; không hard-code trong node generic.
- Chuỗi `0,2,3` phải parse được.
- Chuỗi rỗng phải giữ detect mọi class.

### 6.2. Forward parameter qua `yolo.launch.py`

File `src/yolo_ros/yolo_bringup/launch/yolo.launch.py`.

```python
classes = LaunchConfiguration('classes')
classes_cmd = DeclareLaunchArgument(
    'classes',
    default_value='',
    description='Comma-separated class IDs; empty means all classes',
)
```

Thêm `'classes': classes` vào parameters của `yolo_node`. Thêm `classes_cmd`
vào tuple action trả về từ `run_yolo`.

### 6.3. Tối ưu launch person follower

File `src/yolo_ros/yolo_bringup/launch/person_follower.launch.py`.

```python
classes_arg = DeclareLaunchArgument('classes', default_value='0')
imgsz_height_arg = DeclareLaunchArgument(
    'imgsz_height', default_value='384'
)
imgsz_width_arg = DeclareLaunchArgument(
    'imgsz_width', default_value='640'
)
max_det_arg = DeclareLaunchArgument('max_det', default_value='10')
use_3d_arg = DeclareLaunchArgument('use_3d', default_value='True')
```

Forward vào `yolo.launch.py`:

```python
'classes': LaunchConfiguration('classes'),
'imgsz_height': LaunchConfiguration('imgsz_height'),
'imgsz_width': LaunchConfiguration('imgsz_width'),
'max_det': LaunchConfiguration('max_det'),
'use_3d': LaunchConfiguration('use_3d'),
```

Thêm các argument mới vào `LaunchDescription`.

Cấu hình production đề xuất:

```text
classes=0
imgsz_height=384
imgsz_width=640
max_det=10
use_tracking=True
use_3d=True
use_debug=False
```

Benchmark riêng HandWave có thể dùng `use_3d=False`. Không dùng kết quả đó để
đại diện production nếu follower 3D là bắt buộc.

### 6.4. Khôi phục giới hạn `max_people`

File `src/HandWaveDetection_Pose/hand_wave_detection/ros_node.py`.

Import selector và kiểu bbox:

```python
from raised_hand.logic import (
    NearestTrackSelector,
    RaisedHandRule,
    TemporalRaisedHandFilter,
)
from raised_hand.types import PersonPose, TrackedBox
```

Khởi tạo sau khi load config:

```python
self.selector = NearestTrackSelector(
    max_people=self.config.processing.max_people
)
```

Thay ba list song song `raw_bboxes`, `track_ids`, `confidences` bằng một danh
sách `TrackedBox`. Cách này tránh lệch index giữa bbox, score và ID.

```python
tracked_boxes: List[TrackedBox] = []

for index, detection in enumerate(tracking_msg.detections):
    class_name = detection.class_name.lower()
    is_person = (
        class_name == 'person'
        or detection.class_id == 0
        or 'person' in class_name
    )
    if not is_person or detection.score < confidence_threshold:
        continue

    center_x = detection.bbox.center.position.x
    center_y = detection.bbox.center.position.y
    width = detection.bbox.size.x
    height = detection.bbox.size.y
    bbox = np.asarray(
        [
            center_x - width / 2.0,
            center_y - height / 2.0,
            center_x + width / 2.0,
            center_y + height / 2.0,
        ],
        dtype=np.float32,
    )

    tracked_boxes.append(
        TrackedBox(
            bbox=bbox,
            confidence=float(detection.score),
            track_id=self._parse_track_id(detection.id, index),
        )
    )

selected_boxes = self.selector.select(tracked_boxes, frame.shape)
```

Helper parse ID:

```python
def _parse_track_id(self, raw_id: str, index: int) -> int:
    value = str(raw_id).strip()
    if value.isdigit():
        return int(value)
    if value:
        try:
            return int(value.split('_')[-1])
        except ValueError:
            return abs(hash(value)) % (10**8)
    return -(index + 1)
```

Chỉ đưa bbox đã chọn vào RTMPose. Bắt buộc guard danh sách rỗng:

```python
people: List[PersonPose] = []
if selected_boxes:
    expanded_bboxes = [
        expand_bbox(
            tracked_box.bbox,
            frame.shape,
            self.config.processing.pose_bbox_margin_ratio,
        ).tolist()
        for tracked_box in selected_boxes
    ]
    keypoints_arr, scores_arr = self.pose(
        frame,
        bboxes=expanded_bboxes,
    )
```

Guard này là bắt buộc vì `BatchedRTMPose.__call__` hiện thay `bboxes=[]` bằng
bbox toàn frame. Nếu không có người mà vẫn gọi `self.pose(..., bboxes=[])`, node
sẽ chạy RTMPose toàn ảnh, tạo kết quả sai và tiêu tốn inference time.

Tạo `PersonPose` từ cùng danh sách:

```python
for index, tracked_box in enumerate(selected_boxes):
    keypoints = (
        keypoints_arr[index]
        if index < len(keypoints_arr)
        else empty_pose()[0]
    )
    keypoint_scores = (
        scores_arr[index]
        if index < len(scores_arr)
        else empty_pose()[1]
    )
    people.append(
        PersonPose(
            bbox=tracked_box.bbox,
            confidence=tracked_box.confidence,
            track_id=tracked_box.track_id,
            keypoints=keypoints,
            keypoint_scores=keypoint_scores,
            near_score=tracked_box.near_score,
        )
    )
```

Ràng buộc:

- Phải giữ ID ByteTrack; không đánh lại ID theo thứ tự bbox.
- `TemporalRaisedHandFilter` phụ thuộc ID ổn định qua frame.
- ID âm chỉ là fallback khi tracking chưa cấp ID.
- Không gọi `self.pose` khi `selected_boxes` rỗng.
- `max_people=1` cho FPS tốt nhất nếu chỉ theo người gần nhất.
- `max_people=5` giữ hành vi cũ nhưng làm RTMPose latency cao hơn.

### 6.5. Không render debug khi không có subscriber

Thay lời gọi debug trực tiếp bằng:

```python
if self.debug_pub.get_subscription_count() > 0:
    self._publish_debug_image(
        frame,
        people,
        'rtmpose_external_bbox',
        image_msg,
    )
```

Kỳ vọng:

- Không mở `rqt_image_view`: không vẽ, encode hoặc publish debug image.
- Có subscriber `/pose/image_debug`: debug tự hoạt động.
- Báo cáo benchmark phải ghi rõ debug ON/OFF.

### 6.6. Cho phép override `max_people` từ launch

Nếu cần thay đổi không sửa YAML, thêm launch argument `max_people` và ROS integer
parameter. Không dùng `_string_parameter` cho giá trị số.

```python
def _integer_parameter(self, name: str, default: int) -> int:
    return int(self.declare_parameter(name, default).value)
```

Trong `_load_config`:

```python
max_people = self._integer_parameter(
    'max_people', config.processing.max_people
)
config.processing.max_people = max_people
```

Trong launch:

```python
DeclareLaunchArgument('max_people', default_value='1')
```

Production person-following nên bắt đầu với `max_people=1`. Chỉ tăng khi yêu
cầu nghiệp vụ cần phát hiện vẫy tay từ nhiều người đồng thời.

### 6.7. Sửa `start_person_following.sh`

File `start_person_following.sh`.

Đề xuất biến cấu hình:

```bash
PERSON_MODEL="${PERSON_MODEL:-yolo11n.pt}"
YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"
YOLO_IMGSZ_HEIGHT="${YOLO_IMGSZ_HEIGHT:-384}"
YOLO_IMGSZ_WIDTH="${YOLO_IMGSZ_WIDTH:-640}"
YOLO_MAX_DET="${YOLO_MAX_DET:-10}"
YOLO_CLASSES="${YOLO_CLASSES:-0}"
YOLO_USE_3D="${YOLO_USE_3D:-True}"

HAND_WAVE_BACKEND="${HAND_WAVE_BACKEND:-rtmpose}"
HAND_WAVE_DEVICE="${HAND_WAVE_DEVICE:-cpu}"
HAND_WAVE_MAX_PEOPLE="${HAND_WAVE_MAX_PEOPLE:-1}"
```

Launch YOLO:

```bash
ros2 launch yolo_bringup person_follower.launch.py \
    model:="$PERSON_MODEL" \
    device:="$YOLO_DEVICE" \
    classes:="$YOLO_CLASSES" \
    imgsz_height:="$YOLO_IMGSZ_HEIGHT" \
    imgsz_width:="$YOLO_IMGSZ_WIDTH" \
    max_det:="$YOLO_MAX_DET" \
    use_3d:="$YOLO_USE_3D" \
    input_image_topic:=/camera/color/image_raw \
    input_depth_topic:=/camera/depth/image_raw \
    input_depth_info_topic:=/camera/depth/camera_info
```

Launch HandWave:

```bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
    backend:="$HAND_WAVE_BACKEND" \
    device:="$HAND_WAVE_DEVICE" \
    max_people:="$HAND_WAVE_MAX_PEOPLE" \
    image_topic:=/camera/color/image_raw \
    tracking_topic:=/yolo/tracking
```

Không mặc định RTMPose GPU trước khi ONNX Runtime có
`CUDAExecutionProvider`.

## 7. Implementation giai đoạn 2 — QoS và synchronizer

Chỉ thực hiện sau khi giai đoạn 1 có số liệu.

### 7.1. QoS đề xuất

Image subscriber:

```python
image_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=2,
)
```

Tracking subscriber:

```python
tracking_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=2,
)
```

Không ép `RELIABLE` cho ảnh nếu camera publish `BEST_EFFORT`; QoS không tương
thích có thể khiến subscriber không nhận dữ liệu.

### 7.2. Synchronizer HandWaveDetection

Giá trị khởi đầu:

```python
self.sync = message_filters.ApproximateTimeSynchronizer(
    [self.image_sub, self.tracking_sub],
    queue_size=4,
    slop=0.03,
)
```

Không dùng queue 1 ngay: `/yolo/tracking` đến sau ảnh do YOLO và ByteTrack
latency. Ở 30 FPS, mỗi frame cách nhau khoảng 33 ms.

Ước lượng queue:

```text
minimum_queue >= ceil(p95_tracking_latency_ms / frame_period_ms) + 1
```

Ví dụ p95 tracking latency 70 ms, frame period 33.3 ms: queue tối thiểu khoảng
4.

Do `tracking_node` publish header lấy từ ảnh gốc, có thể thử exact
`TimeSynchronizer` sau khi xác minh stamp:

```bash
ros2 topic echo --once /camera/color/image_raw/header
ros2 topic echo --once /yolo/tracking/header
```

Không chuyển exact synchronization trước khi kiểm tra stamp thực tế.

### 7.3. Synchronizer tracking node

Giảm `slop` từ `0.5` xuống `0.05`, queue khởi đầu 4:

```python
self._synchronizer = message_filters.ApproximateTimeSynchronizer(
    (self.image_sub, self.detections_sub),
    queue_size=4,
    slop=0.05,
)
```

Ràng buộc:

- Detection header lý thuyết trùng ảnh đầu vào.
- Queue quá nhỏ loại ảnh trước khi detection quay lại.
- Queue quá lớn tạo latency khi overload.
- Phải đo p95 latency, không chỉ `ros2 topic hz`.

## 8. Implementation giai đoạn 3 — Latest-frame processing

Áp dụng nếu latency vẫn tăng dần sau giai đoạn 1–2.

Thiết kế:

```text
ROS sync callback
    +-- lưu pair mới nhất
    +-- ghi đè pair cũ chưa xử lý
    +-- return ngay

worker thread
    +-- lấy pair mới nhất
    +-- chạy RTMPose
    +-- publish kết quả
```

State:

```python
self._pending_lock = threading.Lock()
self._pending_pair = None
self._stop_event = threading.Event()
self._worker = threading.Thread(
    target=self._worker_loop,
    daemon=True,
)
self._worker.start()
```

Callback không inference:

```python
def sync_callback(self, image_msg, tracking_msg) -> None:
    with self._pending_lock:
        self._pending_pair = (image_msg, tracking_msg)
```

Worker:

```python
def _worker_loop(self) -> None:
    while not self._stop_event.is_set():
        pair = None
        with self._pending_lock:
            if self._pending_pair is not None:
                pair = self._pending_pair
                self._pending_pair = None

        if pair is None:
            self._stop_event.wait(0.002)
            continue

        self._process_pair(*pair)
```

Shutdown:

```python
def destroy_node(self):
    self._stop_event.set()
    self._worker.join(timeout=2.0)
    return super().destroy_node()
```

Ràng buộc nghiêm ngặt:

- Chỉ một worker gọi `self.pose.session.run`.
- Pair cũ chưa xử lý bị ghi đè; không queue vô hạn.
- Message phải còn hợp lệ trong worker.
- Kiểm thử publish từ worker trên ROS 2 Humble/rclpy thực tế.
- Nếu publish từ worker lỗi, worker tạo kết quả; ROS timer publish.
- Dừng worker trước khi publisher/session bị hủy.

Mục tiêu là dữ liệu mới, không phải xử lý mọi frame camera.

## 9. Implementation giai đoạn 4 — Tối ưu tracking có điều kiện

Không thực hiện như patch đầu tiên.

ByteTrack chủ yếu dùng bbox, score và class. BOTSORT có thể cần image cho GMC
hoặc ReID. Node hiện hỗ trợ cả hai tracker; không được xóa image conversion vô
điều kiện.

Phương án sau khi kiểm tra đúng phiên bản Ultralytics trên Jetson:

```python
self.tracker_requires_image = isinstance(self.tracker, BOTSORT)
```

Trong callback:

```python
tracker_image = None
if self.tracker_requires_image:
    tracker_image = self.cv_bridge.imgmsg_to_cv2(
        img_msg,
        desired_encoding='bgr8',
    )
    tracker_image = cv2.cvtColor(
        tracker_image,
        cv2.COLOR_BGR2RGB,
    )

tracks = self.tracker.update(det, tracker_image)
```

Trước khi merge:

- Pin phiên bản Ultralytics dùng trên Jetson.
- Kiểm tra source `BYTETracker.update` của đúng phiên bản.
- Test ID continuity khi người đi ngang, che khuất ngắn và quay lại.
- Test BOTSORT riêng; không giả định hành vi giống ByteTrack.
- Nếu tracker custom cần ảnh, bổ sung capability flag thay vì `isinstance`.

## 10. Instrumentation bắt buộc

Không tối ưu dựa trên FPS nhìn bằng mắt. Thêm metric hoặc log throttle cho từng
node. Không log mỗi frame.

### 10.1. YOLO node

Đo:

- `predict_ms`.
- `gpu_to_cpu_parse_ms`.
- `publish_rate_hz`.
- `detection_count`.
- Tuổi frame tại đầu callback.

```python
started = time.perf_counter()
results = self.yolo.predict(...)
predict_ms = (time.perf_counter() - started) * 1000.0

parse_started = time.perf_counter()
results = results[0].cpu()
# parse messages
parse_ms = (time.perf_counter() - parse_started) * 1000.0
```

### 10.2. Tracking node

Đo:

- `sync_input_age_ms`.
- `image_conversion_ms`.
- `tracker_update_ms`.
- `tracking_output_rate_hz`.
- Số detection input và tracked output.

### 10.3. HandWaveDetection

Đo:

- `pair_age_ms` tại đầu callback.
- `input_bbox_count` và `selected_bbox_count`.
- `pose_ms`.
- `classification_ms`.
- `debug_render_ms`.
- `callback_total_ms` và `output_rate_hz`.

Tính tuổi frame:

```python
stamp = rclpy.time.Time.from_msg(image_msg.header.stamp)
age_ms = (self.get_clock().now() - stamp).nanoseconds / 1e6
```

Log tổng hợp mỗi 2–5 giây. Log mỗi frame tự làm giảm FPS.

### 10.4. Counter drop và backlog

Nếu dùng latest-frame worker, thêm:

```text
received_pairs
processed_pairs
overwritten_pairs
stale_pairs
```

`overwritten_pairs` tăng là chấp nhận được khi ưu tiên freshness. `pair_age_ms`
tăng liên tục mới là lỗi backlog.

## 11. Quy trình benchmark

### 11.1. Chuẩn hóa môi trường Jetson

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

Ghi lại phiên bản:

```bash
dpkg-query -W nvidia-jetpack
python3 --version
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
python3 -c 'import onnxruntime as ort; print(ort.__version__, ort.get_available_providers())'
python3 -c 'import ultralytics; print(ultralytics.__version__)'
```

Không so sánh hai run khác power mode, thermal state, model cache hoặc dependency
version.

### 11.2. Đo topic rate

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /yolo/detections
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/wave_detected
ros2 topic hz /pose/image_debug
```

`/pose/image_debug` chỉ đại diện output rate khi debug đang bật. Khi debug tắt,
dùng metric nội bộ hoặc `/pose/wave_detected`.

### 11.3. Đo tài nguyên

```bash
sudo tegrastats
```

Theo dõi:

- `GR3D_FREQ`: GPU utilization.
- CPU core utilization.
- RAM và swap.
- EMC/memory controller utilization.
- Nhiệt độ và throttling.

### 11.4. Ma trận benchmark

Mỗi cấu hình warm-up tối thiểu 30 giây, sau đó đo 120 giây.

| ID | YOLO | RTMPose | 3D | Debug | max_people | Mục đích |
|---|---|---|---|---|---:|---|
| A | GPU | CPU | OFF | OFF | 1 | Baseline throughput |
| B | GPU | GPU | OFF | OFF | 1 | Đo GPU contention |
| C | GPU | CPU | ON | OFF | 1 | Production CPU pose |
| D | GPU | GPU | ON | OFF | 1 | Production GPU pose |
| E | GPU | CPU | ON | ON | 1 | Chi phí debug |
| F | GPU | CPU | ON | OFF | 5 | Chi phí multi-person |

Mỗi run ghi:

```text
camera_hz
yolo_detection_hz
yolo_tracking_hz
handwave_hz
p50_latency_ms
p95_latency_ms
YOLO_predict_ms
tracking_ms
RTMPose_ms
debug_ms
GPU utilization
CPU utilization
maximum RAM
temperature
```

### 11.5. Cách xác định bottleneck

- `/yolo/detections` thấp: YOLO/config GPU là bottleneck.
- Detection cao, tracking thấp: synchronizer, image copy hoặc tracker.
- Tracking cao, HandWave thấp: RTMPose, bbox count, debug hoặc callback backlog.
- FPS ổn nhưng `pair_age_ms` tăng: hệ thống xử lý frame cũ.
- GPU gần 100% khi RTMPose GPU: YOLO và RTMPose tranh GPU.
- GPU thấp, CPU/EMC cao: serialization, copy ảnh, tracker hoặc debug.

## 12. Tiêu chí nghiệm thu

### 12.1. Chức năng

- `/yolo/tracking` chỉ chứa person trong launch person follower.
- Track ID không đổi khi người di chuyển bình thường.
- Hand-wave status gắn đúng track ID.
- Không crash khi không có detection.
- Không crash khi detection chưa có ID.
- Không crash khi số detection lớn hơn `max_people`.
- `/person_tracking` vẫn hoạt động khi `use_3d=True`.
- Debug image hoạt động khi có subscriber.

### 12.2. Hiệu năng

- Camera duy trì gần 30 Hz.
- `/yolo/tracking` đạt tối thiểu 20 Hz trong cảnh một người nếu phần cứng cho
  phép.
- HandWaveDetection đạt tối thiểu 15 Hz production.
- Mục tiêu 20–25 Hz với một người, debug OFF.
- `p95` frame age không tăng liên tục.
- Không queue backlog sau 10 phút.
- RAM không tăng liên tục.
- Không thermal throttle trong soak test 10 phút.

### 12.3. Chất lượng detection

- Không giảm recall người gần biên ảnh quá mức khi đổi `480` xuống `384`.
- Không mất target khi confidence dao động quanh threshold.
- `max_det=10` đủ cho môi trường thực tế.
- `max_people` phù hợp use case, không chỉ phù hợp benchmark.

## 13. Ràng buộc và lưu ý

### 13.1. ROS 2 QoS

- Publisher/subscriber QoS phải tương thích.
- Camera thường dùng `BEST_EFFORT`.
- Detection/tracking có thể dùng `RELIABLE`.
- Queue nhỏ giảm latency nhưng có thể mất pair timestamp.
- Queue lớn tăng khả năng ghép nhưng có thể tạo backlog.

### 13.2. Timestamp

- Bbox phải áp dụng lên đúng frame tạo ra detection.
- Không dùng bbox cũ với ảnh mới bất kỳ.
- Sai timestamp làm crop RTMPose lệch khi robot hoặc người di chuyển.
- Latest-frame architecture vẫn phải drop pair quá cũ.

Ngưỡng khởi đầu:

```text
max_pair_age_ms = 150
```

Tune ngưỡng bằng p95 latency thực tế.

### 13.3. Track ID

- Temporal filter phụ thuộc track ID ổn định.
- Không dùng index detection làm ID lâu dài.
- Không hash lại ID nếu message đã chứa số hợp lệ.
- Fallback ID không được giữ temporal state lâu dài.

### 13.4. RTMPose batch

- Batch lớn không luôn có latency thấp hơn.
- ONNX model phải hỗ trợ batch dimension cần dùng.
- Kiểm tra batch 1, 2 và 5.
- Bộ nhớ GPU tăng theo batch.
- Với một target follower, `max_people=1` hợp lý nhất.

### 13.5. RTMPose GPU

Trước khi dùng `device:=cuda:0`:

```bash
python3 - <<'PY'
import torch
import onnxruntime as ort

print('PyTorch CUDA:', torch.cuda.is_available())
print('ORT providers:', ort.get_available_providers())
assert torch.cuda.is_available()
assert 'CUDAExecutionProvider' in ort.get_available_providers()
PY
```

Nếu thiếu `CUDAExecutionProvider`, code hiện tại báo lỗi, không fallback CPU.
Không cài wheel desktop tùy ý trên Jetson ARM64. Wheel phải khớp JetPack, CUDA,
cuDNN, Python ABI và `aarch64`.

### 13.6. Ultralytics version

Trước khi tối ưu tracker internals:

- Ghi version chạy thực tế trên Jetson.
- Pin một version đã test.
- Không dựa trên source desktop khác version production.
- Test lại `BYTETracker.update` và `BOTSORT.update` sau nâng version.

### 13.7. Pipeline 3D

- `use_3d=False` làm `/yolo/detections_3d` ngừng xuất dữ liệu.
- `follower_node` và điều khiển khoảng cách có thể mất input.
- Chỉ tắt 3D khi benchmark hoặc behavior tree không cần distance.

### 13.8. Debug và FPS

- Mở `rqt_image_view` tăng CPU và DDS bandwidth.
- Không dùng FPS cửa sổ debug làm metric duy nhất.
- `ros2 topic hz` tạo subscriber, có overhead nhỏ.
- Log mỗi frame làm sai benchmark.

### 13.9. Build workspace

```bash
export WS="$(git rev-parse --show-toplevel)"
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select yolo_ros yolo_bringup hand_wave_detection
source install/setup.bash
```

Kế hoạch này không đổi message definition. Nếu sau này đổi `yolo_msgs`, build
`yolo_msgs` trước các package phụ thuộc.

## 14. Test plan

### 14.1. Static và build

```bash
python3 -m compileall \
  src/HandWaveDetection_Pose/hand_wave_detection \
  src/HandWaveDetection_Pose/raised_hand \
  src/yolo_ros/yolo_ros/yolo_ros \
  src/yolo_ros/yolo_bringup/launch

colcon build --symlink-install \
  --packages-select yolo_ros yolo_bringup hand_wave_detection
```

### 14.2. Test class filter

- `classes=''`: detect mọi class như cũ.
- `classes='0'`: chỉ person.
- `classes='0,2'`: person và car.
- `classes='abc'`: lifecycle configure phải fail rõ ràng.

### 14.3. Test HandWave

- 0 người: publish `wave_detected=False`, không gọi RTMPose với batch rỗng.
- 1 người có ID: giữ ID qua frame.
- 1 người chưa có ID: fallback không crash.
- 6 người, `max_people=1`: RTMPose nhận batch 1.
- 6 người, `max_people=5`: RTMPose nhận tối đa batch 5.
- Không debug subscriber: `debug_render_ms` gần 0.
- Có debug subscriber: topic ảnh hoạt động.

### 14.4. Test tracking

- ByteTrack giữ ID khi đi ngang.
- Che khuất ngắn không tạo ID mới quá sớm.
- BOTSORT vẫn nhận image nếu được chọn.
- Detection và tracking header giữ timestamp ảnh gốc.

### 14.5. Test soak

Chạy production config ít nhất 10 phút:

```bash
sudo tegrastats
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/wave_detected
```

Không chấp nhận RAM tăng liên tục, frame age tăng liên tục, node restart, CUDA
OOM hoặc thermal throttle kéo dài.

## 15. Thứ tự triển khai

### Patch 1

- Thêm `classes` parameter.
- Person launch dùng `classes=0`.
- YOLO về `640 x 384`.
- `max_det=10`.

Benchmark A–D.

### Patch 2

- Khôi phục `NearestTrackSelector`.
- Áp dụng `max_people` trước RTMPose.
- Debug chỉ render khi có subscriber.

Benchmark A–F.

### Patch 3

- Thêm duration và frame-age metrics.
- Tune queue/slop theo p95 tracking latency.

Chạy soak test 10 phút.

### Patch 4

- Latest-frame worker nếu còn backlog.
- Tối ưu ByteTrack image conversion nếu version thực tế cho phép.

Regression ByteTrack, BOTSORT và person follower 3D.

## 16. Những thay đổi không nên làm ngay

- Không bỏ synchronizer rồi dùng bbox mới nhất với ảnh bất kỳ.
- Không đặt queue 1 trước khi đo tracking latency.
- Không xóa image conversion cho mọi tracker.
- Không tắt 3D production nếu follower cần distance.
- Không chạy nhiều RTMPose worker trên cùng ONNX session.
- Không tăng batch vô hạn để tìm throughput.
- Không mặc định RTMPose GPU chỉ vì provider tồn tại.
- Không đổi model, resolution, tracker và threading trong cùng benchmark.
- Không xóa backend YOLO nội bộ khỏi thư viện cho đến khi pipeline ROS mới qua
  regression; CLI benchmark độc lập vẫn có thể cần backend đó.

## 17. Rollback

Mỗi giai đoạn nên là một commit riêng:

```bash
git log --oneline -n 10
git revert <commit-can-rollback>
```

Không dùng `git reset --hard` trên máy có thay đổi chưa commit.

Fallback runtime không cần revert code:

```bash
# Khôi phục YOLO resolution/max_det cũ
YOLO_IMGSZ_HEIGHT=480 YOLO_MAX_DET=300 \
  bash start_person_following.sh

# Tránh GPU contention
HAND_WAVE_DEVICE=cpu bash start_person_following.sh

# Benchmark không chạy 3D
YOLO_USE_3D=False bash start_person_following.sh
```

## 18. Kết luận

Subscribe `/yolo/tracking` không trực tiếp làm RTMPose chậm. FPS giảm vì pipeline
mới đồng thời có YOLO input lớn hơn, mọi class, `max_det` lớn, tracking tách
process, hai synchronizer, RTMPose xử lý mọi bbox, debug luôn chạy, pipeline 3D
song song và khả năng GPU contention.

Patch có tỷ lệ lợi ích/rủi ro tốt nhất:

```text
person-only YOLO
+ 640x384
+ max_det=10
+ restore max_people
+ conditional debug
+ metric từng tầng
```

Chỉ triển khai latest-frame worker và tracker optimization sau khi số liệu cho
thấy queue, image conversion hoặc backlog vẫn là bottleneck.
