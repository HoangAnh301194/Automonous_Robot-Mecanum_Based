# SOUTA Wakeword — Ứng dụng thu thập dữ liệu độc lập

Chỉ gồm GUI ghi âm và công cụ đóng gói dữ liệu theo nhãn, chạy trên Windows và Ubuntu. Không cần ROS 2, model AI, Kiosk hay code hệ thống robot.

## Windows 10/11 — chạy bằng CMD hoặc Terminal trong VS Code

**Yêu cầu:** cài Python 3.10+ (kiểm tra với `py --version`), Git nếu dùng `git clone`, VS Code nếu muốn mở trong IDE. Cài Python từ python.org; cần bật **Python Launcher** hoặc cấu hình PATH.

### Bước 1. Tải đúng nhánh

Mở **Command Prompt (CMD)** hoặc **Terminal trong VS Code** (Terminal → New Terminal). Nếu đang dùng Terminal VS Code, chọn profile **Command Prompt** từ menu bên cạnh dấu `+` để chạy các lệnh CMD dưới đây.

```bat
git clone --depth 1 --single-branch -b souta-data-collection https://github.com/HoangAnh301194/Automonous_Robot-Mecanum_Based.git SoutaRecorder
cd SoutaRecorder
code .
```

`code .` mở thư mục trong VS Code; nếu máy chưa có lệnh `code`, chọn **File → Open Folder** và mở thư mục `SoutaRecorder`. Có thể tải **Code → Download ZIP** ngay trên GitHub, giải nén rồi mở Terminal tại thư mục chứa `recorder_gui.py`, không cần Git.

### Bước 2. Tạo môi trường Python và cài thư viện (chỉ lần đầu)

Chạy tại thư mục `SoutaRecorder`:

```bat
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Nếu không có lệnh `py`, thử `python -m venv .venv`. Dùng trực tiếp `.venv\Scripts\python.exe` để không cần kích hoạt environment và không gặp lỗi PowerShell Execution Policy.

### Bước 3. Chạy phần mềm thu âm

Mỗi lần mở lại CMD hoặc VS Code Terminal, tại thư mục dự án chỉ cần:

```bat
.\.venv\Scripts\python.exe recorder_gui.py
```

Hoặc chạy **script một lệnh**, tự tạo môi trường/cài thư viện nếu cần:

```bat
start_windows.bat
```

Hai cách hoạt động trong **CMD** và **Terminal VS Code** dùng CMD. Trong **PowerShell của VS Code**, dùng cùng lệnh `.\.venv\Scripts\python.exe recorder_gui.py` hoặc `.\start_windows.bat` (không dùng `activate.bat`).

### Bước 4. Xuất ZIP bằng lệnh (tùy chọn)

GUI đã có nút **Xuất ZIP dataset**, hoặc chạy:

```bat
.\.venv\Scripts\python.exe export_zip.py --collector c01
```

**`c01` là tên viết tắt hoặc mã định danh của người trực tiếp thu thập data**, không phải tên người đang được ghi âm. Ví dụ người thu Hoàng Anh có thể dùng `hoanganh` (không dấu, không khoảng trắng) thay cho `c01`. Tên/mã người thu cần duy nhất và giữ nguyên qua các lần gửi; dùng 2–12 ký tự `a-z`, `0-9`, bắt đầu bằng chữ. ZIP được lưu trong `exports`. Không gửi `data\recorder_registry.json`.

Nếu Windows không nhận microphone, kiểm tra **Settings → Privacy & security → Microphone** và cho phép ứng dụng desktop sử dụng mic.

## Ubuntu Desktop — chạy bằng Terminal hoặc Terminal VS Code

Cài Python/Tkinter và PortAudio:

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip python3-tk libportaudio2
git clone --depth 1 --single-branch -b souta-data-collection \
  https://github.com/HoangAnh301194/Automonous_Robot-Mecanum_Based.git SoutaRecorder
cd SoutaRecorder
```

Lần đầu tạo môi trường, cài thư viện và chạy ứng dụng:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python recorder_gui.py
```

Các lần sau chạy `./.venv/bin/python recorder_gui.py`. Hoặc dùng script `bash start_ubuntu.sh`, script sẽ tự tạo `.venv` và cài thư viện. Xuất ZIP bằng:

```bash
./.venv/bin/python export_zip.py --collector c01
```

Nếu tải ZIP thay vì clone, giải nén và mở Terminal trong thư mục có `recorder_gui.py`. Cần dùng Ubuntu Desktop có microphone và màn hình, không hỗ trợ SSH headless.

## Hướng dẫn thu

1. Chọn nhãn: positive (1, nói “Sô ta” hoặc “Sô ta ơi”); negative (0, câu không có từ khóa); hard_negative (0, gần âm như “sổ tay”, “cô ta”); ambient (0, tiếng ồn không chứa từ khóa).
2. Đăng ký một người nói hoặc phiên môi trường, rồi chọn đúng ID. Cùng một người dùng cùng ID cho tất cả các nhãn.
3. Chọn microphone và thời lượng (positive ~3 giây; ambient 30–60 giây).
4. Bấm Bắt đầu ghi → Nghe lại → Lưu vào dataset hoặc Bỏ bản thu. Chỉ lưu mẫu rõ tiếng, đúng nhãn.
5. File WAV là PCM 16-bit, mono 16 kHz; lưu ở data/raw/<label>/<speaker_or_session_id>/<NNN>.wav.

## Xuất ZIP và gửi người phụ trách

1. Điền **tên viết tắt hoặc mã của người thu thập dữ liệu** ở ô `Tên/mã người thu data` (ví dụ `c01` hoặc `hoanganh`). `c01` chỉ đại diện cho người **thực hiện thu data**, không phải người **đọc từ khóa trong bản ghi**. Mỗi người thu dùng một tên/mã duy nhất và nhất quán; chỉ dùng chữ thường không dấu và số, từ 2–12 ký tự, bắt đầu bằng chữ.
2. Điền tên/mã người thu vào GUI, chọn **Xuất ZIP dataset**, rồi chọn chỗ lưu.
3. Gửi file ZIP qua kênh riêng tư cho người phụ trách, không upload WAV lên GitHub. File ZIP sẽ mang tên/mã người thu để dễ tổng hợp dữ liệu nhiều người.
4. Không gửi data/recorder_registry.json vì file này chứa tên/biệt danh nội bộ.

ZIP giữ nguyên các nhãn như sau:

    dataset/raw/positive/speaker_c01_001/001.wav
    dataset/raw/negative/speaker_c01_001/001.wav
    dataset/raw/hard_negative/speaker_c01_001/001.wav
    dataset/raw/ambient/session_c01_001/001.wav

Có thể xuất ZIP trực tiếp từ Terminal mà không cần kích hoạt môi trường: **Windows** dùng `.\.venv\Scripts\python.exe export_zip.py --collector c01`; **Ubuntu** dùng `./.venv/bin/python export_zip.py --collector c01`. Mặc định tạo ZIP trong `exports/`. Script không ghi đè file ZIP đã có và không xóa WAV gốc.

## Chẩn đoán và kiểm thử

- Windows: kiểm tra quyền mic, chọn đúng thiết bị, xem lỗi trong CMD hoặc Terminal VS Code (hay cửa sổ `.bat`).
- Ubuntu: thiếu Tkinter cài python3-tk; thiếu PortAudio cài libportaudio2.
- Nếu overrun, thử đóng chương trình khác đang chiếm micro và thu lại.
- Kiểm thử không cần micro: Windows dùng `.\.venv\Scripts\python.exe -m unittest discover -p "test_*.py" -v`; Ubuntu dùng `./.venv/bin/python -m unittest discover -p "test_*.py" -v`.
- Chỉ ghi âm khi người tham gia đồng ý; WAV có thể nhận diện người nói, không chia sẻ công khai.

## Files

- recorder_gui.py: GUI ghi âm, nghe lại, lưu/bỏ, xuất ZIP
- recorder_store.py: lưu WAV và registry cục bộ không phụ thuộc microphone
- export_dataset.py: chuẩn hóa ID theo mã cộng tác viên
- export_zip.py: ZIP theo nhãn, không gồm registry
- start_windows.bat / start_ubuntu.sh: chạy nhanh
- requirements.txt, tests: thư viện và kiểm thử
