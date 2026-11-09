---
name: file-edit-permission
description: >-
  Quy tắc kiểm soát quyền chỉnh sửa file trong ros2_ws: Chỉ khi tin nhắn của người dùng bắt đầu bằng dấu chấm (.) thì agent mới được phép chỉnh sửa hoặc tạo file.
---

# Quy tắc bắt buộc: Quyền chỉnh sửa file (Bắt buộc dấu chấm `.` đầu câu)

## 1. Điều kiện tiên quyết để sửa file:
- Agent CHỈ ĐƯỢC PHÉP gọi các công cụ chỉnh sửa hoặc tạo file (`replace_file_content`, `write_to_file`) khi và chỉ khi **tin nhắn yêu cầu của người dùng bắt đầu bằng dấu chấm `.`** (ví dụ: `.sửa file abc`, `.cập nhật xyz`, `.fix lỗi này`).

## 2. Trường hợp KHÔNG có dấu chấm `.` ở đầu câu:
- Dù người dùng có dùng từ khóa "sửa", "thay đổi", "cập nhật", "viết", "tạo file"... nhưng KHÔNG có dấu chấm `.` ở đầu câu (hoặc người dùng có nói là họ quên ghi dấu chấm):
  - **TUYỆT ĐỐI KHÔNG ĐƯỢC GỌI TOOL SỬA FILE.**
  - Agent chỉ được phân tích, giải thích bằng lời, hiển thị đoạn code đề xuất bằng markdown.
  - Lịch sự nhắc người dùng: *"Theo quy ước an toàn của workspace, nếu bạn muốn tôi trực tiếp chỉnh sửa file trên hệ thống, vui lòng gửi lại câu lệnh có dấu chấm `.` ở đầu câu (ví dụ: `.sửa ...`)."*
