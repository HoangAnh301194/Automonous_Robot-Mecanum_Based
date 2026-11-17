# BẢNG TIẾN ĐỘ VÀ ĐỒNG BỘ ĐA AGENT (SHARED TASK BOARD)

> **Mục đích:** Bảng này dùng để các Agent ở các cửa sổ chat độc lập phối hợp với nhau, tránh giẫm chân hoặc đè code lên nhau khi cùng thao tác trên các file dùng chung (`window.py`, `ros_node.py`, `signals.py`...).

## 1. Trạng thái hiện tại của các Cửa sổ Agent (Cập nhật 16/11/2026)

| Module / Cửa sổ | Conversation ID | File đang/đã sửa | Topics / Signals liên quan | Trạng thái | Ghi chú công việc |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SLAM & Theme Sync** | `5b3e0790-8a33-408f-be98-360dbcaafeab` | `slam_page.py`, `navigation_page.py`, `theme.py`, `system_config_page.py`, `window.py` | UI Theme | HOÀN TẤT | Hoàn thành: Đồng bộ màu nền đen chuẩn OLED (#000000, card #0f1318/#111418, button/badge #171c22, border #242c36) cho SLAM, Navigation, System Config; pass 100% linter/build/122 unit tests |
| **System Status & Backend P0-P2 Fixes** | `2e68f068-f68b-4ca8-9296-5a3eb064724f` | `ros_node.py`, `ros2_system.py`, `diagnostics.py`, `sensors_ai.py`, `detection_debug_page.py`, `overview.py`, `admin_page.py`, `contracts.py`, `system_config_page.py`, `test_kiosk_ai.py` | `layer_status_changed`, `system_telemetry_changed`, RQT topics, Cancel Nav | HOÀN TẤT | Hoàn thành toàn diện P0-P2: Sửa layer contract, loại bỏ pump/led test, sửa preflight/nav2/audio, tách cancel nav khỏi e-stop, sửa telemetry AI/water/gpu/wifi, sửa RQT topics, loại bỏ cự ly giả; đã push remote simulator_interface, pull về Jetson Orin, build thành công và pass 100% (122/122) test suite trên Orin. |
| **System Status & Waypoint Sync** | `0619e1a3-457c-4a32-b5c4-64a00e0f7d53` | `navigation_page.py`, `my_map/*_waypoints.yaml`, `test_kiosk_system_status_extended.py` | `apply_map_requested`, `map_changed` | HOÀN TẤT | Hoàn thành: Cơ chế lưu & nạp waypoint riêng theo từng bản đồ (<map_stem>_waypoints.yaml), tự động xóa điểm cũ khi nạp map chưa có điểm, đồng bộ 2 chiều với config/locations.yaml và routes.yaml, pass 100% tests & build |
| **Layer Telemetry, QoS & Overview Badge Sync** | `0619e1a3-457c-4a32-b5c4-64a00e0f7d53` | `overview.py`, `system_config_page.py`, `ros_node.py`, `serial_resolver.py` | `layer_status_changed`, `system_telemetry_changed` | HOÀN TẤT | Hoàn thành: Tích hợp cập nhật card ROS 2 System trên Overview, tắt rviz ở Layer 2, sửa QoS Camera sang reliable, hoàn thiện probe rplidar; colcon build và pass 100% 127 tests trên Jetson Orin |
| **Layer 1 Port Auto-Detect & Orin Build** | `0619e1a3-457c-4a32-b5c4-64a00e0f7d53` | `serial_resolver.py`, `ros_node.py` | `detect_serial_ports`, `start_layer` | HOÀN TẤT | Hoàn thành: Đồng bộ cơ chế tự động dò cổng serial Hoverboard/LiDAR từ gui_v3 vào ros_node.py; đồng bộ và build colcon robot_ui thành công trên Orin; pass 100% 4/4 pytest test_serial_resolver trên Orin. |
| **ROS 2 Layer Live Log Console** | `9a9c04e9-4be4-4f4d-b54a-4917e811fa1d` | `ros2_system.py`, `system_config_page.py`, `ros_node.py`, `signals.py`, `window.py` | `layer_log_received` | HOÀN TẤT | Hoàn thành: Tích hợp khung Live Log Console vào trang ROS 2 System (lọc 5 Layer, auto-scroll, clear log), pipe realtime stdout/stderr từ tiến trình ros_node, pass 100% tests & build. |
| **Admin Configuration** | `a1fd1e54-8359-4442-b74b-70fe27d80e32` | `admin_page.py` | - | HOÀN TẤT | Sắp xếp lại lưới nút chức năng: H1 (System Config, SLAM Config, Navigation), H2 (Detection Debug, Start Nav, Stop Nav) |
| **System Config & Orin Clean Sync** | `a1fd1e54-8359-4442-b74b-70fe27d80e32` | `ros_node.py`, `sensors_ai.py`, `ros2_system.py` | `layer_status_changed`, `slam_status_changed`, Emergency Cleanup, RQT, Serial Resolver | HOÀN TẤT | Sửa lệnh Layer 3 (gọi real_slam thay vì nav2), Layer 4 (use_sim_time:=false), đồng bộ 2 chiều trạng thái SLAM & Layer 3, hoàn thiện cleanup_ros_processes diệt sạch ghost nodes, sửa RQT Image View, bổ sung thông tin cổng Hoverboard/LiDAR theo probe gui_v3, dọn sạch Orin và khởi động lại Kiosk mới nhất (Pass 127/127 tests) |
| **Coordinator & Rules** | `ebc52d35-1122-49c3-89d3-ae990b16cd0e` | `.agent/rules/`, `.agent/` | - | Đang trực | Quản lý quy chuẩn Git, Coding Style, Quality Gate và Điều phối |
| **Kiosk Performance Optimization** | `609aed13-4b65-4add-8353-9e2a6eaaa444` | `ros_node.py`, `window.py`, `navigation_page.py`, `slam_page.py`, `system_config_page.py`, `ros2_system.py`, `home_page.py`, `test_kiosk_performance_optimization.py` | `layer_log_received`, `layer_status_changed`, `map_changed`, `pose_changed` | HOÀN TẤT | Hoàn thành khắc phục 5 điểm tối ưu hiệu năng: (1) Sync log snapshot từ ros_node sang ros2_system khi console active, (2) Lazy map image rebuild khi active, (3) Strictly single visible subpage telemetry trong System Config, (4) STOPPING state & race condition prevention an toàn trong stop_layer/start_layer, (5) OpenCV SIMD resize cho Video Souta. Đạt 137/137 tests pass 100% & colcon build sạch. |

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
