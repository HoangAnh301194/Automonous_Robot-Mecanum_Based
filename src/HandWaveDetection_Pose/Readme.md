# Raised Hand Detection

Hệ thống phát hiện người và xác định độc lập tay trái hoặc tay phải đang được giơ lên. Mục tiêu triển khai trên NVIDIA Jetson Orin Nano với camera Orbbec Astra Pro.

## Môi trường phát triển

Môi trường desktop được cố định ở `CPython 3.14.6`. Tạo lại `.venv` cùng
PyTorch CPU và các dependency đã kiểm thử:

```bash
bash tools/setup_venv.sh
source tools/activate_venv.sh
python --version
```

Script activation đặt thư viện Torch trong `.venv` trước LibTorch hệ thống.
Điều này cần thiết trên máy đang có `/opt/libtorch` trong `LD_LIBRARY_PATH`.

Các phiên bản desktop được khóa trong `requirements-desktop.txt`. Môi trường
Jetson cần cài PyTorch và ONNX Runtime tương thích JetPack riêng, không dùng
wheel CPU trong `tools/setup_venv.sh`.

## Chạy demo CPU

```powershell
source tools/activate_venv.sh
python main.py
```

`config.yaml` mặc định dùng `YOLO11 Detect -> top 5 -> YOLO11 Pose`, thiết bị
`cpu`, camera `0`, detector input `640 x 384` và pose crop `192 x 256`.

Đổi backend mà không sửa file cấu hình:

```powershell
python main.py --backend rtmpose --device cpu
```

## Triển khai ROS 2 Humble trên Jetson Orin (ba camera)

**Áp dụng cho nhánh `main` sau khi đồng bộ topic AI (PR #10).**
Mục này mô tả node ROS 2 `hand_wave_detection`, **khác** với chương trình
`python main.py` dùng để chạy độc lập và benchmark trong các phần bên dưới.
Giữ nguyên namespace của `mo_hinh/three_astra.launch.py`.
Cả ba thiết bị Astra Pro nay đều được định danh bằng serial OpenNI:
Front `17110310855`, Rear `17122514440`, AI `17122611013`.
Với AI RGB/UVC, launch tra serial từ thiết bị Depth rồi ghép giao diện
RGB nằm cùng hub USB nội bộ của camera đó; nếu không ghép được duy nhất,
launch dừng và báo lỗi thay vì lấy nhầm RGB của Rear.
Trước khi chạy lần đầu (đã dừng các camera), nghiệm thu ánh xạ:

~~~bash
cd ~/ros2_ws
python3 src/mo_hinh/launch/_astra_serial_resolver.py \\
  --front-serial 17110310855 \\
  --rear-serial 17122514440 \\
  --ai-serial 17122611013
python3 tools/test_astra_serial_resolver.py
~~~

Giữ nguyên namespace của `mo_hinh/three_astra.launch.py`:

| Camera | Topic ảnh |
| --- | --- |
| AI RGB (UVC, `usb_cam`) | `/ai_camera/color/image_raw` |
| Front Depth (`astra_camera`) | `/front_camera/depth/image_raw` |
| Rear Depth (`astra_camera`) | `/rear_camera/depth/image_raw` |

Luồng AI 2D: **AI RGB → YOLO11 Detection/Tracking (GPU) →
RTMPose (ONNX Runtime CPU) → Skeleton Debug + trạng thái giơ tay**.
Không ghép Depth của Front/Rear vào ảnh AI để suy ra vị trí 3D nếu chưa có
hiệu chuẩn và căn chỉnh tương ứng.

### Chuẩn bị và build (một lần sau khi pull code)

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  mo_hinh yolo_bringup hand_wave_detection

source ~/ros2_ws/install/setup.bash
~~~

Mỗi terminal ROS 2 mới phải chạy `source /opt/ros/humble/setup.bash` và
`source ~/ros2_ws/install/setup.bash`. Các thư viện Python và driver
camera/GPU cần được cài sẵn theo phiên bản JetPack của Jetson.

### Cách A — Tự động chạy toàn bộ pipeline

~~~bash
cd ~/ros2_ws
bash start_person_following.sh
~~~

Script của nhánh `main` khởi chạy ba camera → YOLO trên `cuda:0` →
RTMPose trên `cpu`, tự truyền topic AI mới và `/yolo/tracking`.
Có thể đổi số người xử lý bằng cách đặt biến trước lệnh chạy:

~~~bash
HAND_WAVE_MAX_PEOPLE=1 bash start_person_following.sh
~~~

Các log nằm trong `~/ros2_ws/log/ai_pipeline_YYYYMMDD_HHMMSS/`.
Dừng bằng Ctrl+C; script chỉ dừng các tiến trình nó đã khởi tạo.
Không chạy thêm một bản `three_astra.launch.py` nếu đang dùng script này,
vì hai tiến trình camera sẽ tranh chấp cùng phần cứng.

### Cách B — Chạy riêng từng terminal để kiểm tra

**Terminal 1 — Ba camera:**

~~~bash
ros2 launch mo_hinh three_astra.launch.py \
  front_depth_fps:=15 rear_depth_fps:=15 ai_color_fps:=30
~~~

Giá trị 15 FPS là cấu hình **yêu cầu**; driver Astra có thể fallback sang mode
hỗ trợ khác. Kiểm tra log và tốc độ phát dữ liệu thực tế trước khi kết luận.

**Terminal 2 — YOLO11 nhận ảnh AI, theo dõi người:**

~~~bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.pt \
  device:=cuda:0 \
  input_image_topic:=/ai_camera/color/image_raw \
  image_reliability:=2 \
  classes:=0 \
  imgsz_height:=384 imgsz_width:=640 max_det:=10 \
  use_tracking:=True use_3d:=False use_debug:=True
~~~

**Terminal 3 — RTMPose nhận đúng RGB AI và kết quả Tracking:**

~~~bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  image_topic:=/ai_camera/color/image_raw \
  tracking_topic:=/yolo/tracking \
  max_people:=3
~~~

Các giá trị `image_topic`, `tracking_topic` và `device` trên đây đã
được cập nhật thành **mặc định** trong launch ROS 2; có thể viết gọn:

~~~bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py
~~~

Mặc định **chỉ áp dụng khi chạy đúng file launch phiên bản mới** và không
truyền tham số ghi đè. `max_people` mặc định là 5 nếu không chỉ định.

### Xem ảnh Debug Skeleton

Mở trên Jetson có giao diện đồ họa:

~~~bash
ros2 run rqt_image_view rqt_image_view /pose/image_debug
~~~

Nếu Jetson chạy headless qua SSH, mở video bằng trình duyệt laptop:

~~~bash
sudo apt install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server --ros-args -p port:=8080
~~~

Trên laptop cùng mạng LAN tin cậy truy cập
`http://<IP_JETSON>:8080/stream_viewer?topic=/pose/image_debug`.
Web video server không có cơ chế xác thực mặc định, không công khai ra Internet.

`/pose/image_debug` chỉ được render/phát khi có subscriber, nên hãy mở
trình xem trước khi đo FPS của topic này.

### Kiểm tra topic và xác nhận nguồn ảnh

~~~bash
ros2 node info /yolo/yolo_node
ros2 node info /yolo/tracking_node
ros2 node info /yolo/debug_node
ros2 node info /hand_wave_detection

ros2 topic hz /yolo/tracking
ros2 topic echo /pose/wave_detected
ros2 topic info /ai_camera/color/image_raw -v
ros2 param get /ai_camera/color/usb_cam video_device
~~~

YOLO Detection/Tracking/Debug và HandWaveDetection phải subscribe
`/ai_camera/color/image_raw`; riêng HandWaveDetection nhận thêm
`/yolo/tracking`. Kết quả gồm `/pose/image_debug` (ảnh có khung xương),
`/pose/wave_detected` (Bool), `/pose/wave_status` (String).
Nếu đúng tên topic nhưng hình vẫn là Rear, kiểm tra kết quả ghép serial
và quan sát RGB vật lý. Không tự sửa namespace hoặc đổi sang Depth Rear.

**Lưu ý CUDA:** CUDA của PyTorch dùng cho YOLO và `CUDAExecutionProvider`
của ONNX Runtime dùng cho RTMPose là hai cấu hình độc lập. Trên Jetson hiện
tại, ONNX Runtime chỉ có `CPUExecutionProvider` và
`AzureExecutionProvider`; `device:=cuda:0` cho RTMPose sẽ lỗi khởi tạo.
Dùng `device:=cpu` như lệnh trên. Chỉ chuyển sang CUDA khi đã cài và
nghiệm thu riêng ONNX Runtime GPU tương thích JetPack.

Kiểm tra hồi quy tên topic sau khi cập nhật repo:

~~~bash
cd ~/ros2_ws
python3 tools/test_ai_topic_contract.py
~~~


## Pipeline baseline theo stage

CLI hỗ trợ ba stage độc lập:

```text
detect       YOLO11n Detect + ByteTrack, xuất toàn bộ người
pose         YOLO11 Detect -> top 5 -> YOLO11 Pose hoặc RTMPose
raised_hand  Pose + luật hình học + temporal filter
```

Kiểm tra person detection trước:

```bash
python3 main.py \
  --stage detect \
  --source input.mp4 \
  --output outputs/detect.mp4 \
  --jsonl outputs/detect.jsonl \
  --no-display
```

Trích xuất pose bằng YOLO11 Pose:

```bash
python3 main.py \
  --stage pose \
  --backend yolo11 \
  --source input.mp4 \
  --output outputs/yolo11_pose.mp4 \
  --jsonl outputs/yolo11_pose.jsonl \
  --no-display
```

Trích xuất pose bằng RTMPose:

```bash
python3 main.py \
  --stage pose \
  --backend rtmpose \
  --source input.mp4 \
  --output outputs/rtmpose.mp4 \
  --jsonl outputs/rtmpose.jsonl \
  --no-display
```

Mỗi dòng JSONL chứa `frame_id`, timestamp, backend, latency từng công đoạn,
bbox, `track_id`, detector confidence và toàn bộ keypoint. Stage `detect`
không chạy pose; stage `pose` không chạy luật nhận diện giơ tay.

Tổng hợp và so sánh các file benchmark, bỏ qua frame warm-up đầu tiên:

```bash
python3 tools/summarize_jsonl.py \
  outputs/yolo11_pose.jsonl \
  outputs/rtmpose.jsonl
```

Báo cáo gồm số người mỗi frame, số keypoint hợp lệ, FPS ước lượng và latency
mean/p50/p95 của detector, pose và toàn backend.

## Cấu trúc codebase

```text
main.py                    CLI entry
config.yaml                Cấu hình chạy
raised_hand/app.py         Camera, pipeline, video output
raised_hand/backends.py    YOLO Detect, YOLO top-down pose và RTMPose
raised_hand/config.py      Đọc và kiểm tra YAML
raised_hand/logic.py       Top 5, hình học, temporal filter
raised_hand/types.py       Cấu trúc dữ liệu
raised_hand/export.py      Xuất prediction và latency dạng JSONL
raised_hand/visualization.py
tools/summarize_jsonl.py   Tổng hợp kết quả benchmark
models/                    Model chạy offline
outputs/                   Video kết quả
```

## 1. Mục tiêu

- Phát hiện và theo dõi người trong video.
- Chọn tối đa 5 người gần camera nhất.
- Nhận diện các trạng thái `LEFT`, `RIGHT`, `BOTH`, `NONE`, `UNKNOWN`.
- Tốc độ xử lý tối thiểu 15 FPS.
- `Precision`, `Recall`, `F1-score` của lớp raised-hand đạt tối thiểu 0.90 trên tập kiểm thử thực tế.
- Khoảng cách quan sát mục tiêu tối đa khoảng 15 m.

Không dùng accuracy tổng thể làm tiêu chí chính. Dữ liệu có thể chứa nhiều frame không giơ tay; hệ thống luôn trả về `NONE` vẫn có thể đạt accuracy cao nhưng không phát hiện đúng sự kiện.

## 2. Phần cứng và camera

### NVIDIA Jetson Orin Nano

Jetson chạy TensorRT engine cố định, ưu tiên FP16. INT8 chỉ được dùng sau khi calibration và xác nhận không làm giảm khả năng định vị cổ tay.

### Orbbec Astra Pro

- RGB: `1280 x 720 @ 30 FPS`.
- Depth: `640 x 480 @ 30 FPS`.
- Khoảng đo depth: khoảng `0.6-8 m`, tối ưu trong vùng gần hơn.

Depth không được dùng làm nguồn khoảng cách duy nhất tại 15 m. Hệ thống dùng depth khi hợp lệ; ngoài vùng đo sẽ chuyển sang ước lượng bằng RGB.

## 3. Kiến trúc đề xuất

```text
RGB/Depth Capture
        |
        v
Person Detector
        |
        v
Multi-object Tracker
        |
        v
Distance Estimator
        |
        v
Select Top 5 Nearest Tracks
        |
        v
Batch Top-down Pose Estimation
        |
        v
Raised-hand Geometry Rule
        |
        v
Temporal State Machine
        |
        v
Result: LEFT / RIGHT / BOTH / NONE / UNKNOWN
```

| Thành phần | Lựa chọn | Vai trò |
|---|---|---|
| Person detector | YOLO11n detect | Phát hiện và track người cho backend RTMPose |
| Tracker | ByteTrack | Duy trì `track_id`, giảm tần suất detector |
| Distance estimator | Depth + homography/bbox fallback | Xếp hạng khoảng cách |
| Pose estimator | RTMPose-s `256 x 192` | Trích xuất keypoint cho tối đa 5 ROI |
| Raised-hand classifier | Luật hình học, không train | Phân loại tay trái và tay phải độc lập |
| Runtime | TensorRT FP16 | Suy luận trên Jetson |

Mã nguồn hiện dùng YOLO11n detect cho backend RTMPose để hai backend dùng chung Ultralytics ByteTrack và có thể benchmark công bằng. RTMDet-tiny vẫn là phương án thay thế khi chuyển toàn bộ stack sang OpenMMLab/MMDeploy.

## 4. Luồng xử lý

### 4.1. Capture

- Camera chạy ở `1280 x 720 @ 30 FPS`.
- Hệ thống lấy frame mới nhất và xử lý ở tối thiểu 15 FPS.
- Queue chỉ giữ tối đa một frame; frame cũ bị loại để tránh tích lũy độ trễ.
- RGB là nguồn chính cho detection và pose.
- Depth được căn chỉnh với RGB trước khi đọc khoảng cách.

### 4.2. Person detection

Detector chạy trên toàn bộ frame và chỉ giữ class `person`.

```yaml
input_size: 960x544
confidence_threshold: 0.25
iou_threshold: 0.50
max_detections: 20
```

Nếu vẫn đạt 15 FPS và recall tại 15 m chưa đủ, tăng input lên `1280 x 736`. Detector phải xử lý tất cả người trước khi chọn 5 người gần nhất.

### 4.3. Tracking

ByteTrack cung cấp `track_id` và dự đoán bbox giữa các lần chạy detector.

```text
Frame chẵn: detector -> tracker -> pose
Frame lẻ:   tracker prediction -> pose
```

Nếu chuyển động nhanh hoặc occlusion lớn, detector chạy mỗi frame.

### 4.4. Xếp hạng khoảng cách khi chưa dùng depth

Không dùng riêng diện tích bbox. Dùng điểm kết hợp giữa chiều cao bbox và vị trí chân:

```python
height_score = bbox_height / frame_height
bottom_score = bbox_bottom / frame_height
near_score = 0.75 * height_score + 0.25 * bottom_score
```

`near_score` được làm mượt bằng exponential moving average theo `track_id`. Người có điểm lớn hơn được xem là gần hơn.

Đây là ước lượng tương đối cho prototype. Khi camera được đặt cố định, nên hiệu chỉnh mặt phẳng sàn và dùng bottom-center bbox cùng homography. Homography ổn định hơn bbox size khi chiều cao người, tư thế và mức độ che khuất khác nhau.

### 4.5. Xếp hạng khoảng cách khi deploy Astra Pro

Depth được lấy bằng median trong vùng thân người thay vì toàn bộ bbox:

```text
ROI ngang: 30%-70% chiều rộng bbox
ROI dọc:   20%-75% chiều cao bbox
```

```python
if valid_depth_ratio >= 0.40 and 0.6 <= median_depth <= 8.0:
    distance = median_depth
else:
    distance = homography_or_bbox_estimate
```

Top 5 dùng hysteresis: chỉ thay một track nếu ứng viên mới gần hơn rõ ràng trong nhiều frame liên tiếp.

### 4.6. Pose estimation

Sau khi chọn top 5:

1. Mở rộng mỗi bbox khoảng 10-15%.
2. Crop từng người từ ảnh gốc.
3. Resize mỗi crop thành `256 x 192`.
4. Chạy RTMPose-s cho tối đa 5 crop.
5. Khi deploy TensorRT production, ghép các crop thành batch cố định tối đa 5.

Detector và tracker vẫn theo dõi toàn cảnh; tài nguyên pose chỉ dành cho 5 người cần phân tích.

### 4.7. Phân loại hình học, không cần train

RTMPose và YOLO11 Pose đã được pretrained để trả về keypoint. Hệ thống chỉ dùng quan hệ hình học giữa cổ tay và các điểm khuôn mặt để xác định tay vượt qua đầu.

```python
visible_head_y = min(
    keypoint.y
    for keypoint in [nose, left_eye, right_eye, left_ear, right_ear]
    if keypoint.confidence >= 0.4
)

left_raised = (
    left_wrist_confidence >= 0.4
    and left_elbow_confidence >= 0.4
    and left_shoulder_confidence >= 0.4
    and left_wrist_y < visible_head_y - 0.02 * bbox_height
)
```

Tay phải dùng logic đối xứng.

Nếu không có điểm mặt đủ confidence, trạng thái của tay đó là `UNKNOWN`. Temporal filter xử lý nhiễu ngắn hạn. Không cần dataset hoặc train classifier cho pipeline mặc định.

MLP hoặc fine-tune pose chỉ là phương án dự phòng nếu luật hình học không đạt F1-score mục tiêu trong dữ liệu thực tế.

### 4.8. Temporal filtering

- Bật raised khi đúng ít nhất 3/5 frame gần nhất.
- Tắt raised khi sai ít nhất 5/7 frame gần nhất.
- Keypoint bắt buộc dùng confidence threshold mặc định `0.40`.
- Trả về `UNKNOWN` nếu keypoint bắt buộc có confidence thấp liên tục.

## 5. Tại sao chọn RTMPose?

### 5.1. Phù hợp với top 5 người

RTMPose là top-down pose estimator: mỗi bbox người được crop và resize thành input riêng. Với người ở xa, toàn bộ input pose tập trung vào một người thay vì định vị keypoint của nhiều người trên toàn frame.

Hệ thống chỉ phân tích tối đa 5 người nên các crop có thể được xử lý bằng batch cố định gồm 5 phần tử.

### 5.2. Tách detector và pose

- Tăng detector input khi người 15 m quá nhỏ.
- Đổi RTMPose-s sang RTMPose-m nếu cần thêm độ chính xác.
- Fine-tune detector mà không phải train lại pose.
- Chỉ chạy pose trên top 5 thay vì tất cả người.
- Giữ tracker hoạt động khi tạm thời bỏ qua một lần pose.

### 5.3. Cân bằng tốc độ và độ chính xác

RTMPose sử dụng SimCC để biểu diễn tọa độ keypoint theo hai trục. Thiết kế này hướng tới độ chính xác cao với input pose nhỏ và triển khai thời gian thực.

RTMPose-s là mặc định. RTMPose-m chỉ được dùng nếu RTMPose-s không đạt metric nhưng hệ thống vẫn còn ngân sách thời gian.

### 5.4. Khả năng triển khai

- Có model zoo và cấu hình training trong MMPose.
- Export qua ONNX/MMDeploy sang TensorRT.
- Hỗ trợ batch inference.
- Có thể fine-tune bằng dữ liệu riêng.
- MMPose và RTMPose dùng giấy phép Apache 2.0.

## 6. So sánh các pose model

| Model | Ưu điểm | Hạn chế trong bài toán này | Kết luận |
|---|---|---|---|
| RTMPose-s | Top-down, crop từng người, batch top 5, dễ đổi detector, dễ fine-tune, hỗ trợ TensorRT | Pipeline nhiều thành phần hơn single-model | Lựa chọn chính |
| MediaPipe Pose Landmarker | Dễ tích hợp, 33 landmarks, tốt trên mobile/CPU, có tracking nội bộ | Khó kiểm soát riêng detector và pose; TensorRT trên Jetson không trực tiếp; khả năng fine-tune hạn chế | Phù hợp demo hoặc người gần camera |
| YOLO11 Pose | Dùng chung detector và top 5; batch tối đa 5 crop; export TensorRT thuận tiện | Pose model vẫn có detection head bên trong từng crop, không thuần top-down như RTMPose | Baseline pose thay thế |
| OpenPose | Multi-person bottom-up, phổ biến | Nặng, chậm, kiến trúc cũ | Không chọn |
| HRNet/ViTPose | Độ chính xác keypoint cao | Model lớn, latency và memory cao | Chỉ dùng khi phần cứng còn dư địa |
| MoveNet/BlazePose | Nhẹ, nhanh, phù hợp mobile | Ít linh hoạt hơn với pipeline detector-tracker-top5 | Không ưu tiên |

Hai backend hiện dùng cùng YOLO detector, cùng track và cùng danh sách top 5.
Quyết định cuối phải dựa trên benchmark cùng video, crop và Jetson.

### Khi nào nên chọn YOLO11 Pose?

- Cần pipeline chỉ dùng hệ sinh thái Ultralytics.
- YOLO11 Pose cho keypoint tốt hơn RTMPose trên dữ liệu thực tế.
- Export và vận hành một loại runtime quan trọng hơn độ thuần top-down.
- Batch tối đa 5 crop vẫn đạt đủ 15 FPS và F1-score 0.90.

Nếu YOLO11n-pose đạt toàn bộ metric thực tế, có thể giữ nó để thống nhất
runtime với detector. Cả hai backend đều được giới hạn tối đa 5 người.

### Khi nào nên chọn MediaPipe?

- Ưu tiên CPU hoặc mobile.
- Cần 33 landmark thay vì COCO 17 keypoint.
- Khoảng cách ngắn, người chiếm phần lớn ảnh.
- Không cần fine-tune sâu hoặc TensorRT tùy chỉnh.

MediaPipe vẫn nên được benchmark như baseline. Production hiện ưu tiên khả năng kiểm soát detector, top 5 selection và TensorRT trên Jetson.

## 7. Tối ưu Jetson

Ngân sách cho 15 FPS là `66.7 ms/frame`:

| Công đoạn | Ngân sách thiết kế |
|---|---:|
| Capture và preprocess | 5-8 ms |
| Detector | 15-25 ms |
| Tracker và ranking | 1-3 ms |
| Pose batch 5 | 20-30 ms |
| Classifier và temporal filter | 1-3 ms |
| Output hoặc render | 3-5 ms |

Các giá trị trên là ngân sách mục tiêu, không phải benchmark.

- Build TensorRT engine trên đúng Jetson đích.
- Dùng FP16 cho pose.
- Dùng shape cố định và batch pose tối đa 5.
- Tái sử dụng buffer; tránh cấp phát mỗi frame.
- Không vẽ skeleton trong production nếu không cần.
- Giữ queue capture ở kích thước 1.
- Chỉ dùng INT8 sau khi kiểm tra recall của wrist và elbow.
- Prototype bằng Python; cân nhắc C++/DeepStream khi pipeline ổn định.

## 8. Dataset và đánh giá

Tập dữ liệu phải bao phủ:

- Khoảng cách `0-5 m`, `5-8 m`, `8-15 m`.
- Một tay trái, một tay phải, hai tay, không giơ tay.
- Tay chạm đầu, khoanh tay, chỉ tay, cầm vật và hard negative khác.
- Người quay nghiêng, quay lưng một phần, bị che khuất.
- Ánh sáng trong nhà, ngoài trời, ngược sáng.
- Từ 1 đến nhiều hơn 5 người trong frame.

Chia train/validation/test theo người và phiên quay; không chia ngẫu nhiên các frame gần nhau của cùng video.

```text
Processing FPS            >= 15
Person recall             >= 0.90
Raised-hand precision     >= 0.90
Raised-hand recall        >= 0.90
Raised-hand F1-score      >= 0.90
```

Metric được báo cáo riêng cho từng nhóm khoảng cách. Nếu nhóm `8-15 m` không đạt, ưu tiên dữ liệu đầu vào, detector resolution và vị trí camera trước khi tăng kích thước pose model.

## 9. Lộ trình triển khai

1. Xây baseline bằng detector, ByteTrack và luật hình học.
2. Benchmark RTMPose-s và YOLO11n-pose trên cùng video.
3. Thu và gán nhãn dữ liệu Astra Pro ở ba nhóm khoảng cách.
4. Đánh giá luật hình học; chỉ train classifier nếu không đạt metric.
5. Export detector và RTMPose sang TensorRT FP16.
6. Tích hợp depth và homography fallback.
7. Đo FPS, latency, precision, recall, F1-score trên Jetson.
8. Chỉ chuyển sang model lớn hơn khi xác định đúng bottleneck.

## 10. Tài liệu tham khảo

- [RTMPose paper](https://arxiv.org/abs/2303.07399)
- [MMPose RTMPose model zoo](https://github.com/open-mmlab/mmpose/tree/main/projects/rtmpose)
- [MMDeploy](https://github.com/open-mmlab/mmdeploy)
- [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/)
- [Astra Pro specifications](https://www.proe.vn/astra-pro)

## 11. Chạy và cấu hình hai backend

### Cài đặt cơ bản

```bash
python -m pip install -r requirements.txt
```

Chạy cấu hình mặc định:

```bash
python main.py --config config.yaml
```

Mọi tham số chính nằm trong `config.yaml`. CLI chỉ dùng để override nhanh khi benchmark.

RTMLib cài ONNX Runtime CPU theo dependency mặc định. Trên máy NVIDIA desktop, thay bằng bản GPU:

```bash
python -m pip uninstall -y onnxruntime
python -m pip install onnxruntime-gpu
```

Trên Jetson, không cài tùy ý wheel desktop. Cần cài PyTorch CUDA và ONNX Runtime GPU tương thích đúng phiên bản JetPack đang sử dụng.

Kiểm tra ONNX Runtime:

```bash
python -c 'import onnxruntime as ort; print(ort.get_available_providers())'
```

Kết quả phải chứa `CUDAExecutionProvider` để RTMPose chạy GPU.

### Backend YOLO11 Pose

```bash
python main.py \
  --backend yolo11 \
  --source 0 \
  --device cuda:0
```

```text
YOLO11n detect + ByteTrack
-> xếp hạng bbox
-> chọn top 5
-> mở rộng và batch crop
-> YOLO11n-pose
-> raised-hand rule
-> temporal filter
```

YOLO11 Pose không còn chạy trên toàn frame. Detector chung chọn top 5 trước,
sau đó YOLO11 Pose chạy trên batch crop và keypoint được ánh xạ về ảnh gốc.

### Backend RTMPose

```bash
python main.py \
  --backend rtmpose \
  --source 0 \
  --device cpu
```

```text
YOLO11n detect + ByteTrack
-> xếp hạng bbox
-> chọn top 5
-> RTMPose-s
-> raised-hand rule
-> temporal filter
```

Trên Jetson dùng `--device cpu` trừ khi đã cài ONNX Runtime có `CUDAExecutionProvider`. Bộ tham số dòng lệnh của `main.py` không đồng nghĩa với mặc định của node ROS 2.

Backend này chỉ chạy RTMPose cho 5 người đã chọn. Các crop được ghép thành một batch động và chạy bằng một lần gọi ONNX Runtime; TensorRT là bước tối ưu production tiếp theo.

### Video và lưu kết quả

```bash
python main.py \
  --backend rtmpose \
  --source input.mp4 \
  --device cpu \
  --output outputs/rtmpose.mp4 \
  --no-display
```

Đổi `--backend rtmpose` thành `--backend yolo11` để benchmark cùng video và cùng cấu hình.

### Tham số chính

```text
--max-people 5
--stage detect|pose|raised-hand
--input-width 960
--input-height 544
--pose-input-width 192
--pose-input-height 256
--pose-bbox-margin-ratio 0.12
--confidence 0.25
--iou 0.50
--keypoint-threshold 0.40
--full-precision
--jsonl outputs/predictions.jsonl
```

Mặc định YOLO dùng FP16 khi chạy CUDA. `--full-precision` chuyển YOLO về FP32. RTMPose ONNX chạy theo precision của model ONNX; TensorRT FP16 sẽ được bổ sung ở bước deploy Jetson.
