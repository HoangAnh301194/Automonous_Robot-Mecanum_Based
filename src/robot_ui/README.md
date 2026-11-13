> **Giao diện Kiosk mới (nhánh `simulator_interface`):** Idle Souta Robot → Home PTIT → Chat / Admin PIN. Xem [README_INTERFACE.md](README_INTERFACE.md) để biết luồng màn hình, biến môi trường và cách chạy.

# Robot UI — Souta

Giao diện PyQt5 gồm **Home, Navigation, Chatbot (9router + RAG) & Voice Assistant**.

## 1. Chuẩn bị (sau khi cập nhật code)

### Trên Jetson Orin (`~/ros2_ws`):
```bash
cd ~/ros2_ws
git checkout simulator_interface
git pull origin simulator_interface
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

File cấu hình AI LLM: `config/llm.local.env` (tại thư mục workspace hoặc `~/ros2_ws/config/llm.local.env`):

```dotenv
LLM_BASE_URL=http://127.0.0.1:20128/v1
LLM_API_KEY=your-api-key
LLM_MODEL=your-model-id
```

9router phải được **cài sẵn trên máy chạy Kiosk** (lệnh `9router`
trong `PATH`, hoặc Docker container **đã tạo trước** tên `9router`).
Kiosk sẽ **tự khởi động 9router nếu chưa chạy** và tự đọc file ENV.

## 2. Khởi chạy Kiosk UI

### A. Trên Jetson Orin (Màn hình cảm ứng Robot thực tế):
Mở toàn màn hình, ẩn con trỏ chuột, tối ưu chạm cảm ứng:
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robot_ui kiosk.launch.py
```

### B. Trên máy tính phát triển (Dev / Laptop / PC):
Mở dạng cửa sổ `1024x600`, hiện con trỏ chuột để thao tác:
```bash
source install/setup.bash
ros2 launch robot_ui kiosk.launch.py windowed:=true hide_cursor:=false
```

Bật thêm log debug chi tiết:
```bash
ros2 launch robot_ui kiosk.launch.py windowed:=true hide_cursor:=false debug:=true
```

## 3. Các tính năng Voice & Chatbot mới
- **Nút Micro phóng to 3 lần:** Nằm chính giữa, ngay phía trên thanh nhập câu hỏi với kích thước nút tròn $100\times 100\text{ px}$ (icon $84\times 84\text{ px}$) cùng hiệu ứng hào quang nhịp thở khi đang ghi âm.
- **Tự động thử lại TTS (Auto Retry):** Tự động thử lại khi gặp lỗi mạng tạm thời hoặc `HTTP 502 Bad Gateway` từ upstream server (tối đa 2 lần, cách nhau 0.5s), đảm bảo không bị mất âm thanh câu trả lời.
- **Log tương tác trực quan:** Log chi tiết toàn bộ trạng thái chuyển màn hình, transcript STT, câu trả lời LLM và tiến độ dẫn đường Nav2.

## 4. Dừng & Dọn dẹp tiến trình
Để tắt nhanh Kiosk UI, 9router và dọn dẹp toàn bộ tiến trình ROS 2:
```bash
cd ~/ros2_ws && ./kill_all.sh
```

Hoặc chỉ tắt riêng Kiosk & 9router:
```bash
pkill -9 -f "(robot_ui_kiosk|9router)"
```

Hướng dẫn AI/RAG chi tiết: [README_AI.md](robot_ui/kiosk/README_AI.md).
