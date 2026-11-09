# Agent Working Rules

1. **QUY TẮC BẮT BUỘC VỀ SỬA FILE (BẮT BUỘC CÓ DẤU CHẤM `.` Ở ĐẦU CÂU):**
   - Chỉ khi nào tin nhắn yêu cầu của người dùng **BẮT ĐẦU BẰNG DẤU CHẤM `.`** (ví dụ: `.sửa cái file abc cho tôi`, `.cập nhật ...`) thì AI mới ĐƯỢC PHÉP sử dụng các công cụ chỉnh sửa hoặc tạo file (`replace_file_content`, `write_to_file`).
   - Nếu tin nhắn **KHÔNG có dấu chấm `.` ở đầu câu**, TUYỆT ĐỐI KHÔNG ĐƯỢC PHÉP CHỈNH SỬA HOẶC TẠO BẤT KỲ FILE NÀO, dù người dùng có yêu cầu sửa hay có quên ghi dấu chấm. Trong trường hợp này, AI chỉ được giải thích, hướng dẫn hoặc đề xuất nội dung sửa bằng văn bản và nhắc người dùng thêm dấu chấm `.` ở đầu câu nếu muốn AI trực tiếp chỉnh sửa file.
2. **Giải thích rõ ràng**: Luôn giải thích nguyên nhân, logic và phương án khắc phục chi tiết trước khi tiến hành chỉnh sửa code.
3. **Xin xác nhận từ người dùng**: Luôn hỏi ý kiến và chờ người dùng xác nhận thông tin/kế hoạch trước khi thực hiện bất kỳ chỉnh sửa code hay triển khai nào.
