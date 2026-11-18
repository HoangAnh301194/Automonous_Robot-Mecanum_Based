# SOUTA KWS — HƯỚNG DẪN THU THẬP VÀ GỬI DỮ LIỆU GIỌNG NÓI

> Dành cho thành viên hỗ trợ thu thập dữ liệu từ khóa đánh thức robot **“Sô ta”**. Không cần biết ROS 2 hoặc machine learning. Người phụ trách cung cấp **mã cộng tác viên** và kênh gửi file ZIP riêng tư trước khi bắt đầu.

## 1. Mục tiêu và nguyên tắc

- Thu các câu **có chứa “Sô ta”**, các câu **không chứa từ khóa**, các từ **gần giống**, cùng **tiếng ồn môi trường**.
- Giọng nói **tự nhiên**; không cố kéo dài hai âm, không phát giọng từ loa để thay cho giọng người thật.
- Chỉ ghi âm khi người tham gia đã **đồng ý**. Không ghi thông tin nhạy cảm, số điện thoại, địa chỉ hoặc hội thoại riêng tư.
- File WAV là **dữ liệu giọng nói có thể nhận diện người nói**. **Không upload dữ liệu lên repository mã nguồn công khai** `Automonous_Robot-Mecanum_Based`. Thư mục `data/` đã được `.gitignore` có chủ đích.
- **Không đóng gói `data/recorder_registry.json` vào ZIP**: file này lưu **tên người nói**, chỉ dùng nội bộ trên máy thu.

## 2. Chuẩn bị trên Ubuntu

Cần microphone, Python 3 và mã nguồn Recorder. **Không cần tạo repository riêng để gửi dataset**: dữ liệu sẽ được đóng gói thành ZIP.

```bash
sudo apt update
sudo apt install -y git python3-tk pulseaudio-utils zip unzip

git clone --branch simulator_interface \
  https://github.com/HoangAnh301194/Automonous_Robot-Mecanum_Based.git

cd Automonous_Robot-Mecanum_Based/src/robot_wakeword/training
python3 recorder_gui.py
```

Nếu đã được người phụ trách phát mã nguồn sẵn, chỉ cần vào `src/robot_wakeword/training/` và chạy `python3 recorder_gui.py`. Recorder GUI dùng thư viện chuẩn Python; **không cần tải PyTorch/openWakeWord để thu âm**. Nếu lệnh `python3` không đúng bản, có thể dùng Python trong `.venv`.

Kiểm tra microphone nếu không thu được:

```bash
pactl info | grep 'Default Source'
pactl list short sources
```

**Lưu ý:** Nếu giao diện Recorder chưa hiện hoặc đóng ngay, báo lại cho người phụ trách kèm nội dung Terminal. Không sửa đổi hoặc đưa mã nguồn lên nhánh `simulator_interface` khi chưa được giao nhiệm vụ.

## 3. Đăng ký người nói

1. Chọn label (nhãn) trong GUI.
2. Chọn **Đăng ký mới**, nhập tên hiển thị hoặc biệt danh để phân biệt người thu.
3. GUI sinh ID cục bộ `speaker_001`, `speaker_002`, ...; chọn lại đúng người nói cho các bản ghi tiếp theo.
4. **Một người phải dùng cùng một ID ở cả `positive`, `negative`, `hard_negative`**. Không đăng ký một người thành nhiều ID. Nếu đổi người cầm micro, tạo ID mới.
5. Với `ambient`, đăng ký **phiên ghi môi trường** (`session_001`, ...), không phải người nói.

**Không dùng tên thật làm tên file hoặc thư mục.** `recorder_registry.json` chứa tên hiển thị và **không được gửi kèm ZIP**.

## 4. Chọn nhãn đúng

| Label trong GUI | Nhãn học máy | Nội dung bắt buộc | Thời lượng gợi ý |
|---|---:|---|---|
| `positive` | 1 | Nói đúng **“Sô ta”** hoặc **“Sô ta ơi”**, mỗi file 1 lần | 3 giây |
| `negative` | 0 | Hội thoại/câu nói bình thường, **không có “Sô ta”** | 5–30 giây |
| `hard_negative` | 0 | Các từ dễ nhầm: **“sổ tay”**, “cô ta”, “sô pha”, “số ta”... nhưng **không nói “Sô ta”** | 3–5 giây |
| `ambient` | 0 | Tiếng quạt, điều hòa, động cơ robot, môi trường phòng/lớp học; không có từ khóa | 30–60 giây |

**Đặc biệt:** Một mẫu bị gắn nhãn sai gây hại cho quá trình train hơn là bỏ mẫu đó.

## 5. Ghi âm, nghe lại, hủy, lưu

1. Chọn **người nói/phiên thu** và **label**.
2. Chọn **thời lượng**; bấm **Bắt đầu ghi**.
3. Với `positive` dài 3 giây: im lặng khoảng 0,5 giây → nói **“Sô ta”** một lần ở tốc độ tự nhiên → giữ im lặng đến hết.
4. Có thể bấm **Kết thúc sớm** (sau ít nhất 0,25 giây). Nếu nói nhầm hoặc có tiếng động lớn: bấm **Hủy ghi**.
5. Khi hoàn tất, bấm **Nghe lại** để kiểm tra rõ âm và đúng nội dung.
6. Nếu lỗi, bấm **Bỏ bản thu / Không lưu** rồi ghi lại; nếu đạt, bấm **Lưu vào dataset**.
7. Lặp lại bước 2–6 cho file tiếp theo, **không phải mở lại GUI**.

**Chỉ khi nhấn “Lưu vào dataset”**, file mới được chép vào tập dữ liệu. GUI tự đánh số `001.wav`, `002.wav`… mà không ghi đè file cũ.

### Checklist chất lượng trước khi lưu

- [ ] Đúng người nói và đúng nhãn.
- [ ] Nghe rõ, không bị rè/clipping, micro không quá xa.
- [ ] Với positive: đủ cả hai âm **“Sô”** và **“ta”**, không có thêm câu dài che lấp từ khóa.
- [ ] Với negative/hard negative/ambient: **không xuất hiện** từ khóa thật.
- [ ] Không lẫn giọng người khác hoặc âm nhạc có lời chứa từ khóa.

Thay đổi tự nhiên khoảng cách micro, tốc độ nói, giọng nhỏ/lớn, không gian yên tĩnh và có quạt. Đừng cố tạo mọi mẫu giống hệt nhau.

## 6. Số lượng tham khảo cho mỗi người

Đây là **mục tiêu thu thập ban đầu**, không phải cam kết độ chính xác:

| Nhóm | Đề xuất |
|---|---:|
| Positive | 15–20 mẫu/người (phân bố “Sô ta” và “Sô ta ơi”) |
| Negative | 10–20 mẫu/người (các câu nói tự nhiên) |
| Hard negative | 10–15 mẫu/người |
| Ambient | Một vài phiên 30–60 giây ở các môi trường khác nhau |

Cần **nhiều người nói độc lập** (nên hướng tới 20–30 người trở lên, đa dạng tuổi/giọng/vùng miền). Chỉ thu nhiều lần từ một người **không thay thế** việc thu nhiều người khác nhau. Pipeline hiện yêu cầu **ít nhất 3 người độc lập** có cả positive và negative mới tạo được split train/val/test.

## 7. Dữ liệu được lưu ở đâu?

Recorder lưu tại đường dẫn trong thư mục mã nguồn:

```text
src/robot_wakeword/training/data/
├── recorder_registry.json            # TÊN HIỂN THỊ: KHÔNG UPLOAD
└── raw/
    ├── positive/speaker_001/001.wav
    ├── negative/speaker_001/001.wav
    ├── hard_negative/speaker_001/001.wav
    └── ambient/session_001/001.wav
```

Định dạng mọi WAV: **PCM S16, mono, 16 kHz**. **Không đổi sampling rate hoặc chuyển thành MP3**.

## 8. Xuất dataset theo nhãn, nén ZIP và gửi lại

**Không tạo repository dataset, không cần Git LFS hay thao tác push dữ liệu lên GitHub.** Người ghi âm chỉ cần xuất tập WAV giữ nguyên nhãn, nén **toàn bộ thư mục `dataset/`** thành file `.zip` và gửi lại cho **người phụ trách thu thập/tổng hợp dữ liệu** qua kênh riêng tư đã thống nhất.

### 8.1. Xuất dữ liệu theo đúng nhãn

Người phụ trách cấp **mã cộng tác viên duy nhất** như `c01`, `c02`, `c03`. Cùng một người thu dùng **cùng mã** ở mọi lần gửi, để không trùng `speaker_001` giữa các máy.

Trong thư mục `training`, chạy (thay `c01` bằng mã của mình):

```bash
cd ~/Automonous_Robot-Mecanum_Based/src/robot_wakeword/training
python3 export_dataset.py \
  --collector c01 \
  --dest ~/souta_kws_exports/c01/dataset/raw
```

Script **sao chép WAV, giữ các nhãn** `positive`, `negative`, `hard_negative`, `ambient` và chuyển ID về dạng có mã cộng tác viên. Thư mục đầu ra:

```text
souta_kws_exports/c01/
└── dataset/
    └── raw/
        ├── positive/speaker_c01_001/001.wav
        ├── negative/speaker_c01_001/001.wav
        ├── hard_negative/speaker_c01_001/001.wav
        └── ambient/session_c01_001/001.wav
```

Một người giữ nguyên `speaker_c01_001` trong **mọi nhãn**; tiếng môi trường dùng `session_c01_001`. Script **không xuất `recorder_registry.json`** (có tên người nói), không xóa bản thu gốc và không ghi đè nếu tên trùng nhưng nội dung khác.

### 8.2. Nén nguyên thư mục dataset thành file ZIP

```bash
cd ~/souta_kws_exports/c01
zip -r "souta_kws_c01_$(date +%Y%m%d).zip" dataset/
```

Ví dụ file tạo ra: `souta_kws_c01_20261117.zip`. **Phải nén thư mục `dataset/` chứ không nén từng file WAV rời**, vì cần giữ `dataset/raw/<label>/<speaker_or_session_id>/*.wav`.

Kiểm tra file ZIP có thể giải nén và có đúng nhãn (thay tên file cho phù hợp):

```bash
unzip -t souta_kws_c01_20261117.zip
unzip -l souta_kws_c01_20261117.zip
```

### 8.3. Gửi lại cho người phụ trách thu thập

1. Gửi **file ZIP đã tạo** qua kênh riêng tư do người phụ trách chỉ định (tin nhắn trực tiếp hoặc đường dẫn được giới hạn quyền truy cập).
2. Kèm **mã cộng tác viên**, ngày gửi, số lượng mẫu theo từng nhãn và ghi chú môi trường thu nếu cần. **Không phải gửi tên thật của người nói.**
3. **Không gửi** `recorder_registry.json`; không tải ZIP lên repo công khai hoặc chia sẻ link cho mọi người.
4. Nếu thu thêm dữ liệu, xuất lại bằng **cùng mã cộng tác viên**, tạo ZIP mới và ghi rõ đó là bản cập nhật để người tổng hợp kiểm tra trùng.

## 9. Người phụ trách nhận ZIP và gộp dữ liệu để huấn luyện

Người phụ trách tải ZIP từ kênh nhận riêng tư; kiểm tra cấu trúc, nhãn và WAV trước khi nhập vào pipeline. Ví dụ với file `souta_kws_c01_20261117.zip` được lưu ở `~/Downloads`:

```bash
mkdir -p ~/souta_kws_inbox/c01
unzip -q ~/Downloads/souta_kws_c01_20261117.zip -d ~/souta_kws_inbox/c01

# Tại src/robot_wakeword/training (chứa souta_kws.py)
cp -an ~/souta_kws_inbox/c01/dataset/raw/. data/raw/

env -u LD_LIBRARY_PATH -u PYTHONPATH \
  python souta_kws.py prepare --raw data/raw --manifest data/manifest.csv
```

`cp -an` **không ghi đè file trùng đường dẫn**, nhưng có thể bỏ qua mà không báo lỗi. Nếu có trùng tên, phải đối chiếu nội dung trước khi chấp nhận dữ liệu, không tự ý ghi đè. Giữ lại ZIP gốc để đối soát; **không nhận `recorder_registry.json`**.

`prepare` đọc các folder speaker/session, xác minh WAV và tạo các split **theo danh tính người nói**, không phải ngẫu nhiên theo từng file. Chỉ thực hiện khi tập positive và negative có đủ nhóm độc lập.

**Nhắc lại:** Dữ liệu giọng nói, registry tên và model huấn luyện thuộc tài nguyên nội bộ; tuân thủ sự đồng ý của người nói và quyền truy cập phù hợp.

## 10. Khi có lỗi

| Hiện tượng | Cách xử lý |
|---|---|
| GUI không mở | Kiểm tra `python3-tk`, chạy từ Terminal và gửi log |
| Micro không có tiếng | Kiểm tra `pactl list short sources`, chọn đúng input trong GUI |
| Âm quá nhỏ/rè | Đổi khoảng cách micro, giảm gain nếu clipping, thu lại |
| Ghi nhầm nhãn | **Bỏ bản thu / Không lưu**; chọn nhãn đúng và thu lại |
| Không tạo được ZIP | Kiểm tra `zip`/`unzip`, dung lượng ổ đĩa và đường dẫn thư mục xuất |
| ZIP thiếu nhãn/không giải nén được | Xuất lại qua `export_dataset.py`, nén nguyên thư mục `dataset/` và thử `unzip -t` |
| Trùng file WAV khác nội dung | Không ghi đè; kiểm tra lại mã cộng tác viên và nguồn dữ liệu |

**Liên hệ người phụ trách để được cấp:** `collector_id`, kênh gửi ZIP riêng tư, thời hạn và số mẫu cần thu.
