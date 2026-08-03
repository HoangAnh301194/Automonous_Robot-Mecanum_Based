# Hướng dẫn Đóng gói và Triển khai (Deployment Guide)

Tài liệu này hướng dẫn từng bước đóng gói dự án **HandWaveDetection_Pose** thành file ZIP Portable và triển khai sang các máy tính khác (Windows PC, Linux PC, hoặc NVIDIA Jetson Orin Nano).

---

## 1. Đóng gói Dự án (Trên máy phát triển)

Để tạo file ZIP nén gọn gàng (đã bao gồm đầy đủ weight models, source code và script cài đặt, tự động loại bỏ `.venv` và `outputs/`):

### Trên Windows (Command Prompt / PowerShell):
```powershell
python tools/package_project.py
```

### Trên Linux / macOS:
```bash
python3 tools/package_project.py
```

File ZIP đầu ra sẽ được lưu tại thư mục: `dist/HandWaveDetection_Pose_YYYYMMDD_HHMMSS.zip`.

---

## 2. Triển khai trên máy đích Windows (Windows PC / Laptop)

### Bước 1: Giải nén
- Copy file `.zip` từ thư mục `dist/` sang máy đích.
- Giải nén file ZIP vào một thư mục bất kỳ (ví dụ: `C:\Projects\HandWaveDetection_Pose`).

### Bước 2: Thiết lập môi trường tự động (1-click)
- Mở thư mục đã giải nén.
- Nhấp đôi vào file **`setup_env.bat`** (hoặc chạy `.\setup_env.ps1` trong PowerShell).
- Script sẽ tự động:
  1. Kiểm tra Python.
  2. Tạo môi trường ảo `.venv`.
  3. Cài đặt các thư viện cần thiết từ `requirements-desktop.txt`.

### Bước 3: Chạy ứng dụng
- Nhấp đôi vào file **`run.bat`** (hoặc chạy `.\run.ps1`).
- Hoặc truyền tham số tùy chỉnh:
  ```powershell
  run.bat --backend rtmpose --device cpu
  ```

---

## 3. Triển khai trên NVIDIA Jetson Orin Nano / Linux PC

### Bước 1: Copy & Giải nén
```bash
unzip HandWaveDetection_Pose_*.zip -d HandWaveDetection_Pose
cd HandWaveDetection_Pose
```

### Bước 2: Cấp quyền và Thiết lập môi trường
```bash
chmod +x setup_env.sh run.sh tools/*.sh
./setup_env.sh
```

> **Lưu ý đối với NVIDIA Jetson Orin Nano (ARM64 / JetPack):**
> 1. Nên dùng môi trường ảo kèm `--system-site-packages` (đã được cấu hình mặc định trong `setup_env.sh`) để sử dụng wheel PyTorch CUDA & ONNX Runtime do NVIDIA tối ưu sẵn cho JetPack.
> 2. Kích hoạt môi trường:
>    ```bash
>    source tools/activate_venv.sh
>    ```

### Bước 3: Chạy ứng dụng
```bash
./run.sh
```

Hoặc chạy các stage cụ thể:
```bash
# Chạy với camera trực tiếp
./run.sh --source 0

# Chạy trích xuất pose bằng RTMPose
./run.sh --stage pose --backend rtmpose --source input.mp4 --output outputs/rtmpose.mp4
```

---

## 4. Cấu trúc Gói Triển khai (Dist Package)

```text
HandWaveDetection_Pose/
├── config.yaml               # File cấu hình camera, detector, pose backend
├── DEPLOYMENT.md             # Hướng dẫn triển khai này
├── main.py                   # Điểm vào chính của chương trình
├── Readme.md                 # Tài liệu kỹ thuật chi tiết
├── requirements.txt          # Thư viện phụ thuộc cơ bản
├── requirements-desktop.txt  # Thư viện cố định cho PC
├── setup_env.bat             # Script cài đặt môi trường 1-click (Windows CMD)
├── setup_env.ps1             # Script cài đặt môi trường 1-click (Windows PowerShell)
├── setup_env.sh              # Script cài đặt môi trường 1-click (Linux / Jetson)
├── run.bat                   # Script chạy 1-click (Windows CMD)
├── run.ps1                   # Script chạy 1-click (Windows PowerShell)
├── run.sh                    # Script chạy 1-click (Linux / Jetson)
├── models/                   # Thư mục chứa trọng số đã huấn luyện / ONNX
│   ├── rtmpose-s.onnx
│   ├── yolo11n-pose.pt
│   └── yolo11n.pt
├── raised_hand/              # Source code xử lý logic phát hiện giơ tay
└── tools/                    # Tool đóng gói & benchmark
    ├── activate_venv.sh
    ├── package_project.py
    ├── setup_venv.sh
    └── summarize_jsonl.py
```

---

## 5. Xử lý Lỗi Thường Gặp (Troubleshooting)

1. **Thiếu Python trên Windows**: Tải Python 3.10 - 3.14 từ [python.org](https://www.python.org/) và **tích vào checkbox "Add Python to PATH"** khi cài đặt.
2. **Lỗi camera không mở được**: Kiểm tra index camera trong `config.yaml` (`source: 0`) hoặc truyền tham số `--source 1`.
3. **Môi trường Jetson báo thiếu PyTorch**: Cài đặt PyTorch tương thích JetPack từ trang chủ NVIDIA NVOnline / Jetson PyTorch Wheels trước khi chạy `./setup_env.sh`.
