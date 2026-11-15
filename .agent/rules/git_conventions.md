---
trigger: always_on
description: Quy chuẩn quản lý Git, commit message và quy trình làm việc trên nhánh local
---

# QUY CHUẨN GIT VÀ QUẢN LÝ NHÁNH WORKSPACE

## 1. Môi trường và Nhánh làm việc
- **Nhánh làm việc chính hiện tại:** Mọi chỉnh sửa và commit hiện tại tập trung trên nhánh local `simulator_interface`.
- **Nguyên tắc Remote:**
  - **Tuyệt đối không tự ý push lên remote origin** trừ khi có chỉ định rõ ràng từ người dùng.
  - Khi người dùng yêu cầu push, đảm bảo đã kiểm tra trạng thái branch tracking và không làm ảnh hưởng đến các nhánh khác.

## 2. Quy chuẩn Commit Message
- **Phong cách:** Ngắn gọn, tự do, tập trung vào mô tả chính xác nội dung công việc vừa thực hiện (không ép buộc theo Conventional Commits).
- **Yêu cầu nội dung:**
  - Nêu rõ module hoặc tính năng được thay đổi (ví dụ: `Cập nhật giao diện SLAM kiosk`, `Sửa lỗi routing camera`, `Bổ sung rule coding style`).
  - Viết ngắn gọn, súc tích, tránh commit chung chung như "update", "fix bug".
  - Có thể sử dụng tiếng Việt hoặc tiếng Anh tùy theo ngữ cảnh của task.

## 3. Quy trình trước khi Commit
- Luôn kiểm tra `git status` và `git diff` để đảm bảo:
  - Chỉ commit các file thực sự liên quan đến task hiện tại.
  - Không commit file tạm, file build (`build/`, `install/`, `log/`, `__pycache__/`, `.vscode/`).
  - Đảm bảo code đã vượt qua **Quality Gate** (build và lint thành công) trước khi commit.
