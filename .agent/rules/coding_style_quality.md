---
trigger: always_on
description: Quy chuẩn Coding Style (Python & C++ ROS 2) và Quy định Kiểm soát Chất lượng nghiêm ngặt (Quality Gate)
---

# QUY CHUẨN CODING STYLE VÀ QUALITY GATE (ROS 2 WORKSPACE)

## 1. Coding Style cho Python
- **Tiêu chuẩn:** Tuân thủ chuẩn PEP 8, định dạng rõ ràng, thụt lề 4 spaces.
- **Type Annotations:** Sử dụng Type Hints đầy đủ cho các hàm, phương thức và tham số quan trọng.
- **Xử lý ngoại lệ:** Không dùng `except:` trần (bare except), bắt rõ ràng exception cụ thể và log lại lỗi qua logger của ROS 2 (`self.get_logger().error(...)`).
- **Tài liệu hóa:** Viết docstring ngắn gọn cho class và public method mô tả chức năng, tham số đầu vào và giá trị trả về.
- **Giao diện & UI (PyQt/Kiosk):** Tách bạch rõ luồng giao diện chính (UI Thread) và luồng xử lý nền (Worker/ROS Thread), tránh block main event loop.

## 2. Coding Style cho C++ (ROS 2)
- **Tiêu chuẩn:** Tuân thủ ROS 2 C++ Style Guide (chuẩn C++17 trở lên).
- **Quản lý bộ nhớ:** Sử dụng Smart Pointers (`std::shared_ptr`, `std::unique_ptr`), RAII, tuyệt đối không dùng raw pointer quản lý tài nguyên trừ khi bắt buộc gọi thư viện C legacy.
- **Const Correctness:** Sử dụng `const` và `const &` cho các tham số không thay đổi để tối ưu hiệu năng và an toàn dữ liệu.
- **Header Guards:** Sử dụng `#pragma once` hoặc include guards chuẩn theo đường dẫn package.

## 3. Quy chuẩn Đặt tên trong ROS 2 (Naming Conventions)
- **Node Name:** `snake_case` (ví dụ: `slam_kiosk_node`, `mecanum_base_controller`).
- **Topic Name:** `snake_case` kèm namespace có cấu trúc phân cấp (ví dụ: `/robot/cmd_vel`, `/camera/front/image_raw`).
- **Service & Action Name:** `snake_case` thể hiện hành động (ví dụ: `/navigate_to_pose`, `/save_map_service`).
- **Message / Service / Action Types:** `PascalCase` (ví dụ: `geometry_msgs/msg/Twist`, `nav2_msgs/action/NavigateToPose`).
- **Parameter Name:** `snake_case` có mô tả rõ ràng và giá trị mặc định hợp lý trong file cấu hình `.yaml` hoặc khi khai báo `declare_parameter`.

---

## 4. Quality Gate: Kiểm soát Chất lượng BẮT BUỘC (Strict Mode)

Trước khi Agent báo cáo hoàn thành bất kỳ tác vụ chỉnh sửa hoặc tạo code nào, **BẮT BUỘC** phải thực hiện quy trình kiểm tra sau:

### Bước 1: Kiểm tra Cú pháp & Linter
- **Python:** Chạy linter kiểm tra lỗi cú pháp và style:
  ```bash
  flake8 <đường_dẫn_file_hoặc_package>
  # Hoặc kiểm tra cú pháp nhanh:
  python3 -m py_compile <đường_dẫn_file.py>
  ```
- **C++:** Đảm bảo code sạch, không có warning nghiêm trọng (`ament_cpplint` hoặc kiểm tra trình biên dịch).

### Bước 2: Build Thử nghiệm (Build Verification)
- Chạy `colcon build` cho các package bị ảnh hưởng:
  ```bash
  colcon build --packages-select <tên_package> --symlink-install
  ```
- **Điều kiện:** Quá trình build PHẢI thành công (0 errors).

### Bước 3: Test Verification (nếu có test suite)
- Nếu package có unit test hoặc integration test liên quan:
  ```bash
  colcon test --packages-select <tên_package>
  colcon test-result --verbose
  ```

> [!CAUTION]
> **Quy định hoàn thành task:** Nếu phát hiện lỗi trong quá trình Lint hoặc Build, Agent **KHÔNG ĐƯỢC** báo hoàn thành cho người dùng mà phải tự phân tích nguyên nhân, sửa triệt để lỗi và build pass 100% trước khi kết thúc lượt làm việc.
