---
trigger: always_on
description: Quy chuẩn bắt buộc kiểm tra trạng thái các agent khác và thông báo tiến độ trước khi sửa code dùng chung
---

# QUY CHUẨN ĐỒNG BỘ ĐA CỬA SỔ AGENT (CROSS-AGENT SYNCHRONIZATION)

## 1. Nguyên tắc cốt lõi (BẮT BUỘC)
- Mọi Agent làm việc trong workspace không được hoạt động cô lập.
- Trước khi chỉnh sửa các **file chia sẻ trung tâm** (như `src/robot_ui/robot_ui/kiosk/window.py`, `ros_node.py`, `signals.py`, `camera_mic_resolver.py`):
  1. **PHẢI ĐỌC** bảng tiến độ chung tại: `.agent/SHARED_TASKS.md` (hoặc chạy lệnh `python3 .agent/scripts/scan_active_agents.py`) để biết các agent khác đang phụ trách module nào và đã sửa những gì.
  2. **PHẢI THÔNG BÁO** công việc của mình bằng cách cập nhật dòng trạng thái vào bảng `.agent/SHARED_TASKS.md` trước khi sửa file.

## 2. Quy trình 3 bước khi Agent nhận task

### Bước 1: Kiểm tra xung đột (Pre-check)
- Đọc file `.agent/SHARED_TASKS.md`.
- Nếu thấy có Agent khác đang sửa dở cùng file hoặc liên quan đến module của mình:
  - Đọc kỹ các hàm, signals, topics mà agent đó vừa thêm vào qua `git diff` hoặc nội dung file.
  - Tuyệt đối không xóa, đổi tên hoặc ghi đè lên các thành phần của module khác.

### Bước 2: Thông báo công việc (Announce & Lock)
- Ghi thêm/cập nhật thông tin vào bảng `.agent/SHARED_TASKS.md`:
  - **Thời gian:** Giờ hiện tại.
  - **Cửa sổ / Module:** Tên module đang làm (ví dụ: `Navigation Page`, `SLAM Page`, `System Status`).
  - **Agent ID:** Conversation ID của phiên làm việc.
  - **File tác động:** Các file dự kiến tạo/chỉnh sửa (ví dụ: `window.py`, `navigation_page.py`).
  - **Topics / Signals:** Tên các topic ROS hoặc PyQt Signal định dùng.
  - **Trạng thái:** `Đang thực hiện`.

### Bước 3: Cập nhật khi hoàn tất (Post-update)
- Sau khi hoàn thành code và build thử nghiệm thành công:
  - Cập nhật trạng thái trong `.agent/SHARED_TASKS.md` thành `HOÀN TẤT`.
  - Ghi chú các hàm, Signal hoặc Topic mới vừa tạo để các agent khác biết cách tái sử dụng và tích hợp.
