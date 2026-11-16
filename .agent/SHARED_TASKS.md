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
| **ROS 2 System & Process Log** | `4386e324-f290-434d-a154-1c063a46edc8` | `ros_node.py`, `ros2_system.py`, `signals.py`, `window.py` | `process_log_received`, layer processes | HOÀN TẤT | Hoàn thành: Sửa lỗi Path import shadowing, đồng bộ lệnh khởi chạy 5 layer và dừng tiến trình SIGINT/SIGTERM/SIGKILL theo chuẩn gui_v3, tích hợp khung log console thời gian thực có autoscroll và xóa log trong tab ROS 2 System. Pass 123/123 tests & colcon build. |

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
