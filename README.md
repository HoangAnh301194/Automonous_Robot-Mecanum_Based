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

## Định mức và điều kiện thu — PHẢI ĐỌC TRƯỚC KHI GHI

**Mỗi collector** cần ít nhất **3 speaker độc lập**, mục tiêu **5 speaker**. Chỉ tiêu **tối thiểu 154 file**, mục tiêu **333 file** (bao gồm 4 hoặc 8 bản ambient 30–60 giây). **Không tính nhiều lần thu của cùng một người thành nhiều speaker.**

| Nhãn | Tối thiểu / speaker | Mục tiêu / speaker | Mục tiêu / 5 speaker |
|---|---:|---:|---:|
| `positive` | 20 | 25 | 125 |
| `negative` | 15 | 20 | 100 |
| `hard_negative` | 15 | 20 | 100 |
| `ambient` | 4 phiên / collector | 8 phiên / collector | 8 phiên (30–60 giây) |

- **Khoảng cách**: 0,4–0,6 m (20%), 0,8–1,2 m (50%), 1,5–2,0 m (30%) từ miệng tới microphone.
- **Góc** so với hướng trước microphone: 0°, −30°, +30°, −60°, +60°. Thu cả hai bên, ưu tiên chính diện.
- **Môi trường**: khoảng 50% yên tĩnh, 30% nhiễu nhẹ (quạt, điều hòa), 20% nhiễu vừa (hành lang, tiếng robot, người nói xa đã đồng ý). Mọi nhãn phải có nhiều điều kiện.
- **Mỗi positive / speaker**: 10 lần “Sô ta”, 5 lần “Sô ta ơi”, 10 câu gọi ngắn. **Hard negative**: sổ tay, cô ta, sô pha, soda, số tám, thôi ta, chỗ ta... nhưng **không đọc thành từ gọi thật**.
- **Khi ghi**: chờ 0,3–0,5 giây trước và sau lời nói, chọn 3 giây cho câu ngắn hoặc 5 giây cho câu dài; nghỉ 2–5 giây giữa lượt thu, nghe lại và loại mẫu lỗi.
- **Ambient**: thu 30–60 giây/phiên các nguồn như quạt, điều hòa, động cơ robot, bước chân, bàn phím, cửa, mưa xa, phòng yên tĩnh. Không đọc lời hướng dẫn ambient.

**Tài liệu bắt buộc để giao cho mọi người thu:** [DATA_COLLECTION_PROTOCOL_VI.md](DATA_COLLECTION_PROTOCOL_VI.md) — có bảng quota chi tiết cho từng khoảng cách, từng góc, danh sách tiếng nhiễu, cách đọc, tiêu chí loại mẫu và checklist ZIP.

Trong GUI, bấm **“Hướng dẫn thu chuẩn”** để mở tài liệu này ngay trên phần mềm. Phân bổ trên là **đề xuất cho vòng dữ liệu đầu tiên**, có thể cập nhật sau đánh giá robot thực tế.
## Hướng dẫn thu

1. Chọn nhãn: positive (1, nói “Sô ta” hoặc “Sô ta ơi”); negative (0, câu không có từ khóa); hard_negative (0, gần âm như “sổ tay”, “cô ta”); ambient (0, tiếng ồn không chứa từ khóa).
2. Đăng ký một người nói hoặc phiên môi trường, rồi chọn đúng ID. Cùng một người dùng cùng ID cho tất cả các nhãn.
3. Chọn microphone và thời lượng (positive ~3 giây; ambient 30–60 giây).
4. Bấm Bắt đầu ghi → Nghe lại → Lưu vào dataset hoặc Bỏ bản thu. Chỉ lưu mẫu rõ tiếng, đúng nhãn.
5. File WAV là PCM 16-bit, mono 16 kHz; lưu ở data/raw/<label>/<speaker_or_session_id>/<NNN>.wav.

## Danh sách câu gợi ý có sẵn trong GUI

Người thu **không cần tự nghĩ câu**. Chỉ cần chọn nhãn, đọc đúng câu hiển thị ở mục **“Câu gợi ý để đọc”**, rồi ghi âm và nghe lại. Bấm **“Câu tiếp theo”** để đổi câu; sau khi **lưu** một bản thu, GUI tự chuyển sang câu tiếp theo. Nếu bỏ bản thu, giữ nguyên câu để thu lại. Mỗi nhãn có danh sách riêng, không bị đặt lại khi đổi nhãn.

**Nguyên tắc phân nhãn:** `positive` phải nói đúng tên **“Sô ta”**; `hard_negative` là từ/câu nghe gần giống nhưng **không được đọc thành “Sô ta”**; `negative` là hội thoại bình thường không có từ khóa. `ambient` **không đọc câu trên màn hình** mà chỉ ghi âm thanh môi trường. Nói tự nhiên, đa dạng giọng, khoảng cách và nhịp độ; không cố đọc mọi mẫu giống nhau. Nếu câu dài, chọn 5 giây để tránh cắt âm.

### Positive — gọi robot (26 gợi ý)

1. Sô ta
2. Sô ta ơi
3. Này Sô ta
4. Chào Sô ta
5. Sô ta, nghe không?
6. Sô ta, lại đây
7. Sô ta, giúp mình với
8. Sô ta, bắt đầu nhé
9. Sô ta, dừng lại
10. Sô ta, tiếp tục đi
11. Sô ta, mở bản đồ
12. Sô ta, dẫn đường nhé
13. Sô ta, quay lại
14. Sô ta, đứng yên
15. Sô ta, chào thầy đi
16. Sô ta, tới đây nào
17. Sô ta, cho mình hỏi
18. Sô ta, nghe này
19. Sô ta, trả lời mình
20. Sô ta, nhìn bên này
21. Sô ta, cảm ơn nhé
22. Sô ta, cho tôi hỏi
23. Bạn Sô ta ơi
24. À, Sô ta đây rồi
25. Sô ta, có nghe rõ không?
26. Sô ta ơi, đi thôi

### Hard negative — các câu gây nhầm lẫn (30 gợi ý)

1. Sổ tay
2. Cái sổ tay ở đâu?
3. Tôi để quên sổ tay
4. Cho mình mượn cuốn sổ tay
5. Quyển sổ tay màu đỏ
6. Đừng quên mang sổ tay
7. Cô ta
8. Cô ta đang đứng đó
9. Tôi vừa gặp cô ta
10. Cô ta có tới không?
11. Bạn bảo cô ta đi
12. Cô ta đi qua cửa rồi
13. Sô pha
14. Ghế sô pha màu xanh
15. Ngồi xuống ghế sô pha đi
16. Đổi chiếc sô pha này nhé
17. Soda
18. Cho tôi một lon soda
19. Bạn thích nước soda không?
20. Số tám
21. Đọc số tám nhé
22. Viết số tám lên bảng
23. Số tám ở góc phải
24. Thôi ta đi
25. Thôi ta về nào
26. Chỗ ta đứng rất rộng
27. Cái tô to ở đây
28. Cho tôi cái tô to
29. Còn sót à?
30. Có gì bị sót à?

### Negative — câu thoại không gọi robot (26 gợi ý)

1. Xin chào mọi người
2. Tôi đang ở trong phòng
3. Cho tôi hỏi một chút
4. Bạn có rảnh không?
5. Cửa phòng đã mở chưa?
6. Vui lòng bật đèn lên
7. Chúng ta bắt đầu nhé
8. Robot đang ở đâu?
9. Tôi cần tìm phòng họp
10. Thầy có ở trong phòng không?
11. Hôm nay trời nóng quá
12. Bạn có thể giúp tôi không?
13. Cho tôi xin một cốc nước
14. Mình muốn về phòng thí nghiệm
15. Chờ tôi một chút nhé
16. Bạn đang làm gì đấy?
17. Xin phép đi qua
18. Quay sang bên trái
19. Dừng lại ở đây
20. Đi thẳng tới cuối hành lang
21. Hệ thống đã sẵn sàng
22. Tôi sẽ quay lại sau
23. Chúng mình đi thôi
24. Tôi muốn kiểm tra micro
25. Cảm ơn bạn nhiều
26. Chúc một ngày tốt lành

### Ambient — loại âm môi trường cần ghi (8 gợi ý)

1. Không đọc lời: ghi tiếng quạt đang chạy trong phòng.
2. Không đọc lời: ghi tiếng điều hòa, máy lọc không khí hoặc máy tính.
3. Không đọc lời: ghi tiếng gõ bàn phím hoặc chuột.
4. Không đọc lời: ghi tiếng bước chân ngoài hành lang.
5. Không đọc lời: ghi tiếng cửa mở và đóng tự nhiên.
6. Không đọc lời: ghi tiếng phòng học hoặc phòng họp lúc có người nói chuyện xa.
7. Không đọc lời: ghi tiếng động cơ hoặc bánh xe robot đang chạy.
8. Không đọc lời: ghi tiếng môi trường yên tĩnh, có âm nền tự nhiên.

**Chất lượng thu:** Nên ghi cả tình huống yên tĩnh và có tiếng ồn nhẹ; mỗi người đọc nhiều biến thể, nhưng vẫn cần nhiều người nói khác nhau. Không dùng mẫu âm thanh phát qua loa hoặc biến giọng nhân tạo để thay thế dữ liệu người nói thật.
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

- recorder_gui.py: GUI ghi âm, nghe lại, lưu/bỏ, xuất ZIP và mở hướng dẫn thu chuẩn
- DATA_COLLECTION_PROTOCOL_VI.md: quota/điều kiện khoảng cách, góc, tiếng nhiễu, checklist
- recorder_store.py: lưu WAV và registry cục bộ không phụ thuộc microphone
- export_dataset.py: chuẩn hóa ID theo mã cộng tác viên
- export_zip.py: ZIP theo nhãn, không gồm registry
- start_windows.bat / start_ubuntu.sh: chạy nhanh
- requirements.txt, tests: thư viện và kiểm thử
