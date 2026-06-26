#  Hướng dẫn vận hành Pose Detection ROS 2

Hệ thống này tích hợp 3 thành phần chính: **Camera RealSense** -> **YOLOv8 TensorRT** -> **Pose Gesture Node**.

---

## 1. Khởi động nhanh (Quick Start)

Để bật toàn bộ hệ thống (bao gồm cả Camera, YOLO, Vẫy tay và Face Recognition), bạn chỉ cần dùng script tổng hợp trong workspace:

```bash
cd ~/ros2_ws
./start_person_following.sh
```

---

## 2. Cấu trúc hệ thống & Topic

Hệ thống hoạt động theo chuỗi (Pipeline) sau:

1.  **Camera Node (RealSense):** 
    *   Phát ảnh gốc lên: `/camera/camera/color/image_raw`
2.  **YOLOv8 Node (TensorRT):** 
    *   Nhận ảnh, nhận diện người và điểm xương khớp (Keypoints).
    *   Phát dữ liệu lên: `/yolo/detections`
    *   Phát ảnh debug (có vẽ khung): `/yolo/dbg_image`
3.  **Pose Node (Gesture Recognition):** 
    *   Nhận Keypoints từ YOLO.
    *   Xác định cử chỉ vẫy tay (Hand Raise).
    *   Phát trạng thái lên: `/pose/wave_status`

---

## 3. Các lệnh kiểm tra & Debug

### Xem kết quả vẫy tay :
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /pose/wave_status
```

### Xem hình ảnh debug:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image
```

---

## 4. Cấu hình độ nhạy (Parameters)

Bạn có thể điều chỉnh độ nhạy nhận diện vẫy tay trong file `pose_ros_node.py` hoặc qua tham số ROS 2:

*   `conf_threshold`: Độ tin cậy tối thiểu của điểm xương khớp (mặc định 0.4).
*   `margin`: Khoảng cách tối thiểu cổ tay phải cao hơn mũi (mặc định 20 pixel).

---

## 5. Lưu ý kỹ thuật cho Jetson Orin Nano

*   **Model TensorRT:** File engine `/home/orin/ros2_ws/src/Pose_detection/yolov8n-pose.engine` đã được tối ưu cho GPU.
*   **Kích thước ảnh:** Model yêu cầu đầu vào **640x640**. Hệ thống đã tự động cấu hình resize trong file launch.
*   **Môi trường:** Workspace mặc định là `~/ros2_ws`. File `~/.bashrc` đã được cập nhật để tự động nhận diện workspace này.

---

## 6. Kiến trúc Model (Model Architecture)

Hệ thống sử dụng **YOLOv8-Pose** (cụ thể là phiên bản `yolov8n-pose`) làm lõi nhận diện.
* **YOLOv8-Pose** là mô hình tiên tiến (state-of-the-art) của Ultralytics, có khả năng vừa nhận diện hộp bao (Bounding Box) của người, vừa trích xuất **17 điểm nối xương khớp (Keypoints)** trên cơ thể người trong cùng một thời điểm.
* **Đầu vào (Input):** Ảnh RGB kích thước 640x640.
* **Đầu ra (Output):** Tọa độ (x, y) và độ tin cậy (confidence) của 17 điểm keypoints quan trọng (mũi, mắt, tai, vai, khuỷu tay, cổ tay, hông, đầu gối, mắt cá chân).

---

## 7. Lượng tử hóa (Quantization) & Chuyển đổi định dạng (ONNX/TensorRT)

Để mô hình YOLOv8-Pose có thể chạy mượt mà và đạt FPS cao trên thiết bị nhúng Jetson Orin Nano, model mặc định (`.pt`) đã trải qua quá trình lượng tử hóa và chuyển đổi:

1. **ONNX (Open Neural Network Exchange):** 
   * Trọng số gốc của PyTorch (`.pt`) ban đầu được export sang định dạng ONNX. Đây là định dạng biểu diễn trung gian giúp loại bỏ sự phụ thuộc vào PyTorch framework, tối ưu hóa cho quá trình suy luận (inference) và làm cầu nối cho các engine khác.
2. **TensorRT (Engine):** 
   * Từ file ONNX, hệ thống tiếp tục biên dịch (build) sang định dạng TensorRT engine (`.engine`). Quá trình này biên dịch lại cấu trúc mạng nơ-ron sao cho tối ưu hóa triệt để phần cứng chuyên biệt của kiến trúc GPU Ampere trên Jetson Orin Nano.
3. **Lượng tử hóa (Quantization - FP16 / INT8):**
   * Mặc định, quá trình chuyển đổi TensorRT được áp dụng lượng tử hóa **FP16** (Half-precision floating-point). Phương pháp này giúp giảm một nửa lượng VRAM tiêu thụ và tăng tốc độ xử lý lên đáng kể so với FP32 gốc mà gần như không làm suy giảm độ chính xác của model.
   * *Nâng cao:* Nếu cần tăng tốc tối đa, hệ thống có thể chuyển sang dùng **INT8**. Quá trình INT8 Calibration sẽ yêu cầu một tập dữ liệu (dataset) để hiệu chỉnh nhằm đảm bảo model giữ được độ chính xác sau khi nén dữ liệu.

---

## 8. Logic nhận diện vẫy tay (Wave Hand Logic)

Thuật toán nhận diện hành động vẫy tay hoạt động độc lập tại Pose Node, sử dụng dữ liệu Keypoints trả về từ hệ thống YOLO. Luồng xử lý được thiết kế gồm 2 lớp chính:

**Bước 1: Phân tích không gian trên từng khung hình (Per-frame Logic)**
Trong mỗi khung hình (frame), hệ thống tính toán vị trí tương đối giữa **cổ tay (wrist)** và **mũi (nose)**:
* Một tay được xem là đang "giơ lên" nếu tọa độ Y của cổ tay thấp hơn tọa độ Y của mũi một khoảng bù trừ nhất định (`margin`). (Ghi chú: Trong không gian ảnh, trục Y hướng từ trên xuống dưới).
* **Công thức điều kiện:** `wrist_y < (nose_y - margin)` VÀ `confidence >= threshold`.
* Quá trình này được quét đồng thời cho cả hai tay (tay trái và tay phải).

**Bước 2: Máy trạng thái làm mịn theo thời gian (Temporal Smoothing State Machine)**
Dữ liệu xương khớp đôi khi bị nhiễu, làm cho tọa độ bị giật chớp nhoáng. Để tránh hiện tượng chập chờn báo động (flickering), hệ thống kết hợp một State Machine (Máy trạng thái):
* **Chuyển từ IDLE ➔ WAVING:** Hành động giơ tay phải được duy trì liên tục và thỏa mãn điều kiện không gian trong ít nhất `N` khung hình (ví dụ: 5 frames liên tiếp).
* **Chuyển từ WAVING ➔ IDLE:** Tay phải được hạ xuống dưới ngưỡng trong ít nhất `M` khung hình (ví dụ: 8 frames liên tiếp).
* Chỉ khi State Machine chuyển sang trạng thái ổn định là `WAVING`, hệ thống mới xuất (publish) message báo hiệu lên topic `/pose/wave_status` để các node khác có thể sử dụng (ví dụ: kích hoạt theo người, chào hỏi, v.v.).
