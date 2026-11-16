---
trigger: always_on
description: Quy chuẩn phân công, giao tiếp và phối hợp giữa Coordinator và các Subagents trong workspace
---

# QUY CHUẨN PHỐI HỢP ĐA AGENT (SUBAGENT COLLABORATION)

## 1. Phân định Trách nhiệm (Role & Responsibility)
- **Coordinator (Agent chính):**
  - Chịu trách nhiệm tổng thể: Tiếp nhận yêu cầu người dùng, phân rã bài toán thành các sub-task độc lập, kiểm tra chất lượng và báo cáo kết quả cuối cùng.
  - Tuyệt đối không giao các task phụ thuộc lẫn nhau cho nhiều subagent chạy song song mà không có thứ tự đồng bộ.
- **Subagent (Agent con):**
  - **Giới hạn phạm vi (Scope Boundary):** Chỉ tập trung thực thi đúng vai trò được mô tả trong Prompt/Role (ví dụ: Researcher chỉ đọc/phân tích, Coder chỉ sửa file chỉ định, Tester chỉ chạy build/test).
  - Không tự ý mở rộng phạm vi công việc ra ngoài task được giao.

## 2. Quản lý Workspace & Tránh Xung đột File (Workspace Isolation)
- **Tác vụ chỉ đọc / nghiên cứu (Research):** Sử dụng `Workspace: "inherit"`.
- **Tác vụ viết code song song (Parallel Coding/Building):** 
  - BẮT BUỘC sử dụng `Workspace: "share"` (working tree độc lập) hoặc `"branch"` (clone độc lập) để tránh ghi đè đồng thời lên cùng một file nguồn của workspace chính.
  - Khi subagent hoàn thành, Coordinator có trách nhiệm kiểm tra diff và merge lại vào workspace chính một cách an toàn.

## 3. Giao tiếp & Cơ chế Đánh thức (Reactive Communication)
- **Không Polling:** Antigravity sử dụng cơ chế Reactive Wakeup tự động. Sau khi `invoke_subagent` hoặc gửi `send_message`, Coordinator KHÔNG ĐƯỢC chạy vòng lặp hỏi trạng thái liên tục; chỉ cần nhường lượt và chờ hệ thống tự đánh thức khi subagent hoàn thành.
- **Nội dung giao việc (Briefing):** Khi gọi subagent, Coordinator phải cung cấp đầy đủ thông tin:
  1. Mục tiêu cụ thể của sub-task (Goal).
  2. Các file/thư mục được phép đọc/ghi.
  3. Định nghĩa hoàn thành rõ ràng (Definition of Done).

## 4. Quy chuẩn Bàn giao Kết quả (Handoff Protocol)
Mọi Subagent khi hoàn tất nhiệm vụ BẮT BUỘC phải gửi tin nhắn báo cáo lại cho Coordinator theo cấu trúc:
1. **Kết quả đạt được:** Tóm tắt ngắn gọn những gì đã hoàn thành.
2. **Danh sách thay đổi:** Các file đã tạo, sửa hoặc xoá (kèm đường dẫn).
3. **Quality Gate:** Trạng thái linter/build/test (đã pass 100% hay còn warning gì).
4. **Lưu ý / Rủi ro:** Các vấn đề còn tồn đọng hoặc điểm Coordinator cần lưu ý.

## 5. Thu dọn Tài nguyên (Cleanup)
- Sau khi Subagent hoàn thành và bàn giao kết quả xong, Coordinator có trách nhiệm thu dọn subagent (`manage_subagents` với `Action: "kill"`) để giải phóng tài nguyên hệ thống, trừ khi subagent đó được chỉ định duy trì để tiếp tục trao đổi.
