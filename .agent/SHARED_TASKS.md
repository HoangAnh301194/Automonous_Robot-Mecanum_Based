# BẢNG TIẾN ĐỘ VÀ ĐỒNG BỘ ĐA AGENT (SHARED TASK BOARD)

> **Mục đích:** Bảng này dùng để các Agent ở các cửa sổ chat độc lập phối hợp với nhau, tránh giẫm chân hoặc đè code lên nhau khi cùng thao tác trên các file dùng chung (`window.py`, `ros_node.py`, `signals.py`...).

## 1. Trạng thái hiện tại của các Cửa sổ Agent (Cập nhật 16/11/2026)

| Module / Cửa sổ | Conversation ID | File đang/đã sửa | Topics / Signals liên quan | Trạng thái | Ghi chú công việc |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SLAM & Theme Sync** | `5b3e0790-8a33-408f-be98-360dbcaafeab` | `slam_page.py`, `navigation_page.py`, `theme.py`, `system_config_page.py`, `window.py` | UI Theme | HOÀN TẤT | Hoàn thành: Đồng bộ màu nền đen chuẩn OLED (#000000, card #0f1318/#111418, button/badge #171c22, border #242c36) cho SLAM, Navigation, System Config; pass 100% linter/build/122 unit tests |
| **Hardware AI Telemetry** | `2e68f068-f68b-4ca8-9296-5a3eb064724f` | `contracts.py`, `system_config_page.py`, `hardware.py` | `HostTelemetrySnapshot` | HOÀN TẤT | Hoàn thành: Cập nhật kiểm tra 9Router (port 20128) & Voice STT (Groq Whisper), loại bỏ Cloud Gemini, hiển thị siêu gọn 0 icon, pass 100% tests & build |
| **System Status** | `0619e1a3-457c-4a32-b5c4-64a00e0f7d53` | `system_config_page.py`, `hardware.py`, `sensors_ai.py`, `detection_debug_page.py`, `navigation_page.py`, `slam_page.py`, `window.py` | `/diagnostics`, `/yolo/detections`, `apply_map_requested` | HOÀN TẤT | Hoàn thành: Tích hợp Process Monitor (CPU/RAM), nút RQT cam, tạo DetectionDebugPage (Pose Debug RQT + Telemetry + Face placeholder), đồng bộ map YAML vào NavigationPage, đồng bộ icon minimize/maximize toàn bộ trang |
| **Admin Configuration** | `a1fd1e54-8359-4442-b74b-70fe27d80e32` | `admin_page.py` | - | HOÀN TẤT | Sắp xếp lại lưới nút chức năng: H1 (System Config, SLAM Config, Navigation), H2 (Detection Debug, Start Nav, Stop Nav) |
| **Coordinator & Rules** | `ebc52d35-1122-49c3-89d3-ae990b16cd0e` | `.agent/rules/`, `.agent/` | - | Đang trực | Quản lý quy chuẩn Git, Coding Style, Quality Gate và Điều phối |
| **Patrol & Approach Control** | `4386e324-f290-434d-a154-1c063a46edc8` | `bt_nodes.py`, `mission_controller_node.py`, `nav2_params.yaml` | `ActionApproachWavingPerson`, `/tracked_people` | HOÀN TẤT | Hoàn thành: Hạ cự ly tiếp cận người vẫy tay xuống 0.45m (freeze_distance 0.75m), người chắn đường xuống 0.70m, siết xy_goal_tolerance còn 0.22m; pass 100% 13 unit tests, build pass 0 errors. |
| **UI Performance & Lazy Sleep** | `4386e324-f290-434d-a154-1c063a46edc8` | `ros_node.py`, `detection_debug_page.py`, `system_config_page.py`, `window.py` | `robot_status_changed`, `system_telemetry_changed` | HOÀN TẤT | Hoàn thành: Chặn bão pin 50Hz về 1Hz, chỉ render trang active, cho tab Debug & System Config ngủ 100% khi ẩn, pass 100% linter/build/125 unit tests |
| **2D Pose Estimate in Navigation** | `4386e324-f290-434d-a154-1c063a46edc8` | `navigation_page.py`, `ros_node.py`, `window.py`, `test_kiosk_navigation_page.py` | `/initialpose`, `initial_pose_requested` | HOÀN TẤT | Hoàn thành: Thêm nút '📍 2D ESTIMATE', cho phép bấm và kéo chuột chỉ hướng robot trên bản đồ tương tự RViz, phát PoseWithCovarianceStamped tới /initialpose; pass 100% linter/build/unit tests |
| **ESP32 RC Remote Limiter** | `4386e324-f290-434d-a154-1c063a46edc8` | `firmware/esp32_bridge/src/main.cpp` | RC CH1, RC CH2 | HOÀN TẤT | Hoàn thành: Giới hạn tốc độ tiến lùi 0.8 m/s (RC_MAX_SPEED 95) và góc lái an toàn 0.8 rad/s (RC_MAX_STEER 40), PlatformIO build pass 100%. |
| **System Map Select & Nav 2D Pose** | `4386e324-f290-434d-a154-1c063a46edc8` | `ros2_system.py`, `system_config_page.py`, `window.py`, `navigation_page.py` | `map_selected`, `initial_pose_requested` | HOÀN TẤT | Hoàn thành: Tập trung chọn bản đồ mục tiêu trực tiếp trong System Config (ROS 2 Layer Screen), tự động đồng bộ sang NavigationPage và ros_node với đường dẫn tuyệt đối, hướng dẫn 2D Pose Estimate để AMCL định vị tức thì; pass 100% build và unit tests |

---

## 2. Quy ước sử dụng dành cho Agent
1. **Trước khi bắt đầu sửa code:**
   - Mở bảng này ra đọc để xem module của mình có liên quan hoặc phụ thuộc vào module nào đang được làm không.
   - Bổ sung hoặc cập nhật thông tin dòng của mình với trạng thái `Đang thực hiện`.
2. **Khi sửa file chia sẻ trung tâm (`window.py`, `ros_node.py`):**
   - Chỉ import và khởi tạo module của mình, tuyệt đối không chỉnh sửa hoặc xóa code của các module khác.
   - Thống nhất các tên Signal/Topic để tránh xung đột namespace.
3. **Khi hoàn tất:**
   - Cập nhật trạng thái thành `HOÀN TẤT` và ghi chú tóm tắt các component đã thêm.
