# SOUTA Wakeword — Ứng dụng thu thập dữ liệu độc lập

Chỉ gồm GUI ghi âm và công cụ đóng gói dữ liệu theo nhãn, chạy trên Windows và Ubuntu. Không cần ROS 2, model AI, Kiosk hay code hệ thống robot.

## Windows 10/11

1. Cài Python 3.10+ từ python.org (bật Python Launcher hoặc PATH).
2. Trên GitHub chọn nhánh souta-data-collection → Code → Download ZIP, giải nén.
3. Nhấp đúp **start_windows.bat** để tự tạo .venv, cài thư viện và mở GUI.
4. Nếu thiếu micro, cho phép microphone trong Settings → Privacy & security → Microphone.

## Ubuntu Desktop

Mở terminal:

    sudo apt update
    sudo apt install -y git python3 python3-venv python3-tk libportaudio2
    git clone --depth 1 --single-branch -b souta-data-collection https://github.com/HoangAnh301194/Automonous_Robot-Mecanum_Based.git SoutaRecorder
    cd SoutaRecorder
    bash start_ubuntu.sh

Hoặc tải ZIP nhánh, giải nén, mở terminal trong thư mục và chạy bash start_ubuntu.sh. Phải dùng máy có Desktop GUI và microphone.

## Hướng dẫn thu

1. Chọn nhãn: positive (1, nói “Sô ta” hoặc “Sô ta ơi”); negative (0, câu không có từ khóa); hard_negative (0, gần âm như “sổ tay”, “cô ta”); ambient (0, tiếng ồn không chứa từ khóa).
2. Đăng ký một người nói hoặc phiên môi trường, rồi chọn đúng ID. Cùng một người dùng cùng ID cho tất cả các nhãn.
3. Chọn microphone và thời lượng (positive ~3 giây; ambient 30–60 giây).
4. Bấm Bắt đầu ghi → Nghe lại → Lưu vào dataset hoặc Bỏ bản thu. Chỉ lưu mẫu rõ tiếng, đúng nhãn.
5. File WAV là PCM 16-bit, mono 16 kHz; lưu ở data/raw/<label>/<speaker_or_session_id>/<NNN>.wav.

## Xuất ZIP và gửi người phụ trách

1. Xin mã cộng tác viên duy nhất từ người phụ trách (ví dụ c01).
2. Điền mã vào GUI, chọn **Xuất ZIP dataset**, rồi chọn chỗ lưu.
3. Gửi file ZIP qua kênh riêng tư cho người phụ trách, không upload WAV lên GitHub.
4. Không gửi data/recorder_registry.json vì file này chứa tên/biệt danh nội bộ.

ZIP giữ nguyên các nhãn như sau:

    dataset/raw/positive/speaker_c01_001/001.wav
    dataset/raw/negative/speaker_c01_001/001.wav
    dataset/raw/hard_negative/speaker_c01_001/001.wav
    dataset/raw/ambient/session_c01_001/001.wav

Có thể dùng lệnh python export_zip.py --collector c01 (sau khi kích hoạt .venv). Mặc định tạo ZIP trong exports/. Script không ghi đè file ZIP đã có, không xóa dữ liệu WAV gốc.

## Chẩn đoán và kiểm thử

- Windows: kiểm tra quyền mic, chọn đúng thiết bị, xem lỗi trong cửa sổ chạy .bat.
- Ubuntu: thiếu Tkinter cài python3-tk; thiếu PortAudio cài libportaudio2.
- Nếu overrun, thử đóng chương trình khác đang chiếm micro và thu lại.
- Kiểm thử không cần micro: python -m unittest discover -p "test_*.py" -v.
- Chỉ ghi âm khi người tham gia đồng ý; WAV có thể nhận diện người nói, không chia sẻ công khai.

## Files

- recorder_gui.py: GUI ghi âm, nghe lại, lưu/bỏ, xuất ZIP
- export_dataset.py: chuẩn hóa ID theo mã cộng tác viên
- export_zip.py: ZIP theo nhãn, không gồm registry
- start_windows.bat / start_ubuntu.sh: chạy nhanh
- requirements.txt, tests: thư viện và kiểm thử
