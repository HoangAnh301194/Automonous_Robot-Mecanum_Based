> **Giao diện Kiosk mới (nhánh `simulator_interface`):** Idle Dasai Mochi → Home PTIT → Chat / Admin PIN. Xem [README_INTERFACE.md](README_INTERFACE.md) để biết luồng màn hình, biến môi trường và cách chạy.

# Robot UI — Souta

Giao diện PyQt5 gồm **Home, Navigation, Chatbot (9router + RAG)**.

## 1. Chuẩn bị (sau khi cập nhật code)

```bash
cd ~/ros2_ws
git switch simulator
git pull --ff-only origin simulator
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_ui robot_bringup mo_hinh
source install/setup.bash
```

File cấu hình riêng trên Jetson: `~/ros2_ws/config/llm.local.env`

```dotenv
LLM_BASE_URL=http://127.0.0.1:20128/v1
LLM_API_KEY=your-api-key
LLM_MODEL=your-model-id
```

9router phải được **cài sẵn trên máy chạy Kiosk** (lệnh `9router`
trong `PATH`, hoặc Docker container **đã tạo trước** tên `9router`).
Kiosk sẽ **tự khởi động 9router nếu chưa chạy** và tự đọc file ENV,
không cần terminal riêng hay `source llm.local.env`.
Nếu 9router ở máy khác, Kiosk chỉ kết nối tới IP trong `LLM_BASE_URL`,
không thể khởi động server từ xa.

## 2. Chạy (mỗi lệnh ở một terminal)

**Terminal 1 — GUI v3 (mở trước):**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/gui_v3.py
```

Trong GUI v3, chọn chế độ **Simulation/Robot thật**, map và khởi động
Layer 1, Layer 2, Nav2 theo nhu cầu.

**Terminal 2 — Backend:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup backend.launch.py
```

**Terminal 3 — Kiosk Souta (tự khởi động 9router):**

Robot thật:

```bash
ros2 launch robot_ui kiosk.launch.py windowed:=true hide_cursor:=false use_sim_time:=false
```

Gazebo:

```bash
ros2 launch robot_ui kiosk.launch.py windowed:=true hide_cursor:=false use_sim_time:=true
```

**Lưu ý:** GUI v3 hiện dọn một số tiến trình ROS 2/Gazebo khi
khởi động và đóng cửa sổ, nên hãy mở GUI v3 **trước** Backend/Kiosk.

## 3. Kiểm tra nhanh

```bash
ros2 topic list | grep -E '^/(map|map1|robot_status|navigation/status)$'
ros2 action list | grep navigate_to_pose
tail -n 40 ~/.ros/9router-kiosk.log  # khi 9router do Kiosk tự mở
```

- GUI v3 đọc `/map1`, Kiosk đọc `/map`. Nếu cần, nhấn **Áp dụng & Publish** trong GUI v3.
- Log `9router config loaded` chỉ xác nhận đọc ENV; hãy gửi một câu hỏi thử trong Chat.
- Nếu Chat báo **HTTP 503**, kiểm tra model, provider và quota trong 9router.
- Nếu 9router không tự mở: kiểm tra `command -v 9router` hoặc `docker ps -a`.
- Có thể tắt tự mở bằng `auto_start_9router:=false`, hoặc chỉ định
  `router_command:="/đường/dẫn/9router --no-browser"` khi chạy Kiosk.

Hướng dẫn AI/RAG thêm: [README_AI.md](robot_ui/kiosk/README_AI.md).
