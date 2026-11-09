---
trigger: always_on
description: Chỉ cho phép chỉnh sửa hoặc tạo file khi tin nhắn của người dùng bắt đầu bằng dấu chấm (.)
---

# BẮT BUỘC: QUYỀN CHỈNH SỬA VÀ TẠO FILE TRONG WORKSPACE

1. **Điều kiện sửa file:**
   - AI CHỈ ĐƯỢC PHÉP thực hiện bất kỳ hành động chỉnh sửa hoặc tạo file nào (thông qua `replace_file_content`, `write_to_file`, hay chạy lệnh bash có thao tác ghi đè/sửa file) khi tin nhắn yêu cầu của người dùng **BẮT ĐẦU BẰNG DẤU CHẤM `.`** (ví dụ: `.sửa cái file abc cho tôi`).

2. **Nếu tin nhắn KHÔNG có dấu chấm `.` ở đầu câu:**
   - **NGHIÊM CẤM TỰ Ý SỬA FILE**, bất kể nội dung câu hỏi là gì và dù người dùng có nói là họ quên ghi dấu chấm đi chăng nữa.
   - AI chỉ được giải thích, hướng dẫn, đưa ra nội dung code mẫu trong khung phản hồi văn bản, và nhắc người dùng gõ thêm dấu chấm `.` ở đầu câu nếu muốn AI áp dụng sửa file.
