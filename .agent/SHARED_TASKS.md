# BẢNG TIẾN ĐỘ VÀ ĐỒNG BỘ ĐA AGENT (SHARED TASK BOARD)

> **Mục đích:** Bảng này dùng để các Agent ở các cửa sổ chat độc lập phối hợp với nhau, tránh giẫm chân hoặc đè code lên nhau khi cùng thao tác trên các file dùng chung (`window.py`, `ros_node.py`, `signals.py`...).

## 1. Trạng thái hiện tại của các Cửa sổ Agent (Cập nhật 16/11/2026)

| Module / Cửa sổ | Conversation ID | File đang/đã sửa | Topics / Signals liên quan | Trạng thái | Ghi chú công việc |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SLAM & Map Manager** | `5b3e0790-8a33-408f-be98-360dbcaafeab` | `slam_page.py` | `/map`, `/scan` | Đang thực hiện | Sửa từ ngữ "bản đồ" -> "map", bàn phím ảo, cảnh báo trùng tên |
| **Navigation Page** | `2e68f068-f68b-4ca8-9296-5a3eb064724f` | `navigation_page.py`, `window.py` | `/plan`, `/nav_status`, `fullscreen_toggle_requested` | HOÀN TẤT | Hoàn thành NavigationPage: 0 emoji, icon ảnh PNG thực tế, chuẩn hóa typography 4 cấp, fullscreen toggle, badges Wifi & Pin, đồng bộ YAML 2 chiều, pass 100% tests & build |
| **System Status** | `0619e1a3-457c-4a32-b5c4-64a00e0f7d53` | `system_config_page.py`, `window.py` | `/diagnostics`, `/battery_state`, `system_telemetry_changed` | HOÀN TẤT | Hoàn thành SystemConfigPage: 0 emoji, dùng icon có sẵn, pass build & tests |
| **Admin Configuration** | `a1fd1e54-8359-4442-b74b-70fe27d80e32` | `admin_page.py` | - | HOÀN TẤT | Sắp xếp lại lưới nút chức năng: H1 (System Config, SLAM Config, Navigation), H2 (Detection Debug, Start Nav, Stop Nav) |
| **Coordinator & Rules** | `ebc52d35-1122-49c3-89d3-ae990b16cd0e` | `.agent/rules/`, `.agent/` | - | Đang trực | Quản lý quy chuẩn Git, Coding Style, Quality Gate và Điều phối |

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
