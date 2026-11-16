# SOUTA KWS — HƯỚNG DẪN THU THẬP VÀ GỬI DỮ LIỆU GIỌNG NÓI

> Dành cho thành viên hỗ trợ thu thập dữ liệu từ khóa đánh thức robot **“Sô ta”**. Không cần biết ROS 2 hoặc machine learning. Người phụ trách cung cấp **mã cộng tác viên** và quyền truy cập **repository dataset PRIVATE** trước khi bắt đầu.

## 1. Mục tiêu và nguyên tắc

- Thu các câu **có chứa “Sô ta”**, các câu **không chứa từ khóa**, các từ **gần giống**, cùng **tiếng ồn môi trường**.
- Giọng nói **tự nhiên**; không cố kéo dài hai âm, không phát giọng từ loa để thay cho giọng người thật.
- Chỉ ghi âm khi người tham gia đã **đồng ý**. Không ghi thông tin nhạy cảm, số điện thoại, địa chỉ hoặc hội thoại riêng tư.
- File WAV là **dữ liệu giọng nói có thể nhận diện người nói**. **Không push vào repository mã nguồn công khai** `Automonous_Robot-Mecanum_Based`. Repository này đã `.gitignore` thư mục `data/` có chủ đích.
- Không push `data/recorder_registry.json`: file đó lưu **tên người nói**, chỉ dùng nội bộ trên máy thu.

## 2. Chuẩn bị trên Ubuntu

Cần microphone, Python 3 và quyền truy cập 2 repository: **repository mã nguồn** và **repository dataset riêng ở chế độ PRIVATE** (đường dẫn do người phụ trách cung cấp).

```bash
sudo apt update
sudo apt install -y git git-lfs python3-tk pulseaudio-utils

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

**Không dùng tên thật làm tên file hoặc thư mục.** `recorder_registry.json` chứa tên hiển thị và không được gửi qua Git.

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

## 8. Gửi dataset lên Git — dùng repository PRIVATE riêng

Người phụ trách phải tạo một repository **Private** trên GitHub, ví dụ `souta-kws-dataset`, và cấp quyền **Write** cho cộng tác viên. Cần bật Git LFS để lưu WAV; không push dữ liệu vào repository mã nguồn công khai.

### 8.1. Người phụ trách thiết lập một lần

Trong repository dataset PRIVATE (đã clone trên máy):

```bash
cd ~/souta-kws-dataset
git lfs install
git lfs track '*.wav'
# File .gitattributes phải được commit để mọi người cùng có quy tắc LFS
git add .gitattributes
git commit -m 'chore: track WAV with Git LFS'
git push origin main
```

Nếu GitHub repository vẫn chưa có file nào hoặc nhánh mặc định không phải `main`, người phụ trách cần khởi tạo nhánh tương ứng trước khi gửi lệnh cho nhóm. **Không gửi token, password hoặc private SSH key vào nhóm chat.**

### 8.2. Mỗi cộng tác viên xuất dữ liệu và đẩy lên

Người phụ trách cấp **mã cộng tác viên duy nhất** như `c01`, `c02`, `c03`. Cùng một người thu phải giữ nguyên mã này ở mọi lần xuất. Mã được dùng để tránh trùng `speaker_001` của hai máy khác nhau.

Clone private dataset repo bằng URL mà người phụ trách cung cấp:

```bash
cd ~
git clone <PRIVATE_DATASET_REPO_URL> souta-kws-dataset
cd souta-kws-dataset
git lfs install
```

Sau đó quay về **thư mục training của Recorder** và chạy (thay `c01` bằng mã của mình):

```bash
cd ~/Automonous_Robot-Mecanum_Based/src/robot_wakeword/training
python3 export_dataset.py \
  --collector c01 \
  --dest ~/souta-kws-dataset/data/raw
```

Lệnh chỉ **copy WAV**, không upload tên người nói; tự chuyển ID như sau:

```text
Nguồn trên máy thu:  data/raw/positive/speaker_001/001.wav
Trong repo PRIVATE: data/raw/positive/speaker_c01_001/001.wav

Nguồn trên máy thu:  data/raw/negative/speaker_001/001.wav
Trong repo PRIVATE: data/raw/negative/speaker_c01_001/001.wav
```

Một người giữ nguyên `speaker_c01_001` trong **mọi label**, nhờ đó pipeline không trộn giọng cùng người giữa Train/Validation/Test. Với tiếng môi trường, script dùng dạng `session_c01_001`.

Script **không ghi đè** file đã tồn tại: file giống thì bỏ qua; file trùng tên nhưng khác dữ liệu thì báo lỗi để kiểm tra. Không xóa dữ liệu gốc.

Kiểm tra kỹ repository đang là **PRIVATE** trước khi push:

```bash
cd ~/souta-kws-dataset
git remote -v
git status --short
git add .gitattributes data/raw
git commit -m 'data(kws): add recordings c01'
git pull --rebase origin main
git push origin main
```

Nếu `git pull --rebase` gặp conflict, **dừng và báo người phụ trách**, không force push. Lặp lại bước xuất và commit/push khi có thêm bản thu mới. Cần kiểm tra hạn mức lưu trữ/băng thông Git LFS của repo.

> Nếu không có quyền truy cập GitHub private hoặc mạng kém, gửi bản WAV bằng kênh lưu trữ **riêng tư** được người phụ trách chỉ định. Không nén rồi đưa lên repository mã nguồn công khai.

## 9. Người phụ trách gộp dữ liệu để huấn luyện

Trên máy train, clone/pull **repository dữ liệu private**, sau đó chép riêng `data/raw/` vào `src/robot_wakeword/training/data/raw/` của pipeline. Không lấy `recorder_registry.json` của bất kỳ cộng tác viên nào.

```bash
# Ở thư mục training chứa souta_kws.py
cp -an ~/souta-kws-dataset/data/raw/. data/raw/

env -u LD_LIBRARY_PATH -u PYTHONPATH \
  python souta_kws.py prepare --raw data/raw --manifest data/manifest.csv
```

`prepare` đọc các folder speaker/session, xác minh WAV và tạo các split **theo danh tính người nói**, không phải ngẫu nhiên theo từng file. Chỉ thực hiện khi tập positive và negative có đủ nhóm độc lập.

**Nhắc lại:** Dữ liệu giọng nói, registry tên và model huấn luyện thuộc tài nguyên nội bộ; tuân thủ sự đồng ý của người nói và quyền truy cập phù hợp.

## 10. Khi có lỗi

| Hiện tượng | Cách xử lý |
|---|---|
| GUI không mở | Kiểm tra `python3-tk`, chạy từ Terminal và gửi log |
| Micro không có tiếng | Kiểm tra `pactl list short sources`, chọn đúng input trong GUI |
| Âm quá nhỏ/rè | Đổi khoảng cách micro, giảm gain nếu clipping, thu lại |
| Ghi nhầm nhãn | **Bỏ bản thu / Không lưu**; chọn nhãn đúng và thu lại |
| Không thể push Git | Kiểm tra quyền Write, Git LFS, nhánh, và gửi toàn bộ thông báo lỗi |
| Trùng file WAV khác nội dung | Không ghi đè; kiểm tra lại mã cộng tác viên và nguồn dữ liệu |

**Liên hệ người phụ trách để được cấp:** `collector_id`, private dataset repository URL, quyền Write, thời hạn và số mẫu cần thu.
