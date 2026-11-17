# QUY TRÌNH CHUẨN THU THẬP DỮ LIỆU WAKEWORD “SÔ TA”

> Dành cho người thu thập dữ liệu (collector) trên Windows/Ubuntu. Đây là **định mức khởi đầu do dự án đề xuất**, không phải tiêu chuẩn đảm bảo mô hình đạt một mức accuracy cố định. Nếu microphone thật của robot có phạm vi thu khác, người phụ trách cần điều chỉnh sau khi đánh giá thực nghiệm.

## 1. Phân biệt hai vai trò

- **Collector (người thu data)**: người vận hành ứng dụng, đặt microphone, hướng dẫn người đọc và xuất ZIP. Mã collector ví dụ `c01` hoặc `hoanganh`.
- **Speaker (người nói)**: người trực tiếp đọc các câu. Một collector phải mời **nhiều speaker độc lập**, không ghi một người rồi đổi tên để đủ chỉ tiêu.
- `speaker_001` trong máy là một người nói; khi xuất ZIP thành `speaker_c01_001`. Người nói đó phải dùng **cùng ID** ở cả `positive`, `negative`, `hard_negative`.
- Nếu cùng một speaker ghi ở **hai collector khác nhau**, phải báo người tổng hợp để ánh xạ cùng một danh tính trước khi chia train/validation/test, tránh **speaker leakage**.

## 2. Số mẫu bắt buộc và mục tiêu

| Nhãn | Tối thiểu / speaker | Mục tiêu / speaker | Tối thiểu / collector (3 speaker) | Mục tiêu / collector (5 speaker) |
|---|---:|---:|---:|---:|
| `positive` | 20 | 25 | 60 | 125 |
| `negative` | 15 | 20 | 45 | 100 |
| `hard_negative` | 15 | 20 | 45 | 100 |
| `ambient` | — | — | 4 phiên × 30–60 giây | 8 phiên × 30–60 giây |
| **Tổng file** | **50 / speaker** | **65 / speaker** | **154** | **333** |

**Mục tiêu ưu tiên**: 5 speaker thực sự khác nhau, mỗi speaker 25 positive + 20 negative + 20 hard negative; collector tự thu ít nhất 8 clip ambient dài 30–60 giây. Không bắt buộc cân bằng số file ambient với các file speech vì thời lượng mỗi loại khác nhau.

**Mỗi speaker nên có** các cách phát âm/độ tuổi/chất giọng/vùng miền khác nhau; ưu tiên có cả giọng nam/nữ và nhiều ngữ điệu tự nhiên. Không thu toàn bộ bộ dữ liệu từ duy nhất một người, kể cả có hàng trăm mẫu.

## 3. Chuẩn bị trước khi thu

1. Xin **đồng ý ghi âm** của từng speaker; không ghi người lạ không biết mình đang bị thu. Tránh thu thông tin riêng tư trong lời nói.
2. Dùng laptop/microphone USB hoạt động tốt. **Ưu tiên đúng loại micro của robot** hoặc thu thêm bộ mẫu bằng chính microphone robot nếu khả thi. Micro laptop và microphone robot có đáp ứng tần số khác nhau.
3. Đặt microphone **cố định**, không cầm lắc tay, không chạm vào bàn/micro trong khi ghi. Nếu robot có microphone ở cao độ xác định, cố gắng dựng micro ở cao độ vận hành tương đương.
4. Đánh dấu trên sàn bằng băng dính các mốc **0,5 m; 1,0 m; 1,8 m** đo từ miệng người nói tới capsule mic. Giữ một hướng mặt trước microphone làm chuẩn **0°**.
5. Chọn ít nhất **2 địa điểm** có đặc tính âm học khác nhau (ví dụ phòng học và hành lang/phòng thí nghiệm); nếu có thể chọn 3.
6. Kiểm tra bản thu thử 3 giây: có giọng người rõ, không méo, không quá nhỏ; trong GUI chọn đúng nguồn micro.

## 4. Khoảng cách từ miệng tới microphone

| Nhóm | Khoảng cách | Tỷ lệ mong muốn | Positive / speaker (25) | Negative / speaker (20) | Hard negative / speaker (20) |
|---|---|---:|---:|---:|---:|
| Gần | 0,4–0,6 m | khoảng 20% | 5 | 4 | 4 |
| Trung bình | 0,8–1,2 m | khoảng 50% | 12 | 10 | 10 |
| Xa | 1,5–2,0 m | khoảng 30% | 8 | 6 | 6 |
| **Tổng** | | **100%** | **25** | **20** | **20** |

- Đây là **khoảng cách theo không gian 3D từ miệng tới microphone**, không phải khoảng cách giữa các lần bấm ghi.
- 0,8–1,2 m là khoảng cách tương tác thường dùng trong giai đoạn khởi đầu. Nếu robot phục vụ cần nhận người từ 2–3 m, có thể thêm mẫu xa **sau khi kiểm tra chất lượng mic**; không bắt buộc ghi xa đến mức giọng không thể nghe rõ.
- **Mỗi nhãn đều phải có cả ba khoảng cách**. Không được thu tất cả `positive` ở gần và mọi `negative` ở xa: mô hình có thể học khoảng cách thay cho từ khóa.

## 5. Góc thu / hướng người nói (theo mặt phẳng ngang)

**Định nghĩa:** 0° là người nói đứng chính diện hướng mặt trước microphone; +30°/+60° ở bên phải và −30°/−60° bên trái so với trục trước mic. Đây là **góc vị trí tương đối**, không phải bắt buộc quay cổ đúng số độ. Giữ chiều cao miệng tự nhiên, không cần đo pitch/yaw bằng thiết bị chuyên dụng.

| Góc vị trí | Positive / speaker (25) | Negative / speaker (20) | Hard negative / speaker (20) |
|---|---:|---:|---:|
| 0° (chính diện) | 11 | 8 | 8 |
| −30° (lệch trái nhẹ) | 4 | 4 | 4 |
| +30° (lệch phải nhẹ) | 4 | 4 | 4 |
| −60° (lệch trái lớn) | 3 | 2 | 2 |
| +60° (lệch phải lớn) | 3 | 2 | 2 |
| **Tổng** | **25** | **20** | **20** |

- Không cần cố định từng câu với một góc. Sau vài bản thu, đổi vị trí/đổi góc **luân phiên giữa cả ba nhãn**.
- Phần lớn mẫu nên để người nói hướng miệng về microphone; thi thoảng quay đầu nhẹ sang trái/phải để mô phỏng sử dụng thực tế.
- Góc 90° hoặc người nói quay lưng chỉ là **mẫu bổ sung thử nghiệm**, không nên chiếm phần lớn dữ liệu khi chưa biết robot có thu rõ không.

## 6. Tiếng nhiễu và điều kiện môi trường khi thu giọng

| Điều kiện | Tỷ lệ gợi ý | Positive / speaker (25) | Negative / speaker (20) | Hard negative / speaker (20) |
|---|---:|---:|---:|---:|
| Yên tĩnh: chỉ có noise floor của phòng | 50% | 13 | 10 | 10 |
| Nhiễu nhẹ: quạt, điều hòa, PC | 30% | 8 | 6 | 6 |
| Nhiễu vừa: hành lang, giọng nói xa có đồng ý, tiếng robot | 20% | 4 | 4 | 4 |
| **Tổng** | **100%** | **25** | **20** | **20** |

**Danh sách nguồn tiếng nhiễu khuyến nghị (ghi thật, không giả lập bằng cách chỉnh WAV):**

1. Quạt điện, quạt trần, quạt laptop.
2. Điều hòa, máy lọc không khí, tiếng gió quạt.
3. Động cơ, bánh xe, tiếng di chuyển của robot.
4. Bàn phím, chuột, thao tác với máy tính.
5. Tiếng bước chân, cửa mở/đóng, ghế kéo trong hành lang.
6. Người trò chuyện ở xa **đã đồng ý ghi âm**, không để ai trong nền nói đúng “Sô ta”.
7. Tiếng máy in, thiết bị phòng thí nghiệm hoặc tiếng máy hoạt động liên tục.
8. Tiếng xe cộ xa ngoài cửa, tiếng mưa/gió tự nhiên mức nhẹ.
9. Phòng có tiếng vang tự nhiên (phòng lớn, hành lang, cửa kính).
10. Phòng yên tĩnh/tiếng nền rất thấp để kiểm soát baseline.

**Mức nhiễu:** tiếng nhiễu phải nhỏ hơn hoặc không lấn át hoàn toàn lời nói, người nghe vẫn nghe rõ “Sô ta”. Không bắt buộc đo SNR bằng đồng hồ; **không tăng độ nhiễu quá cao chỉ để đủ quota**. Tránh nhạc có lời/TV hoặc file ghi âm phát từ loa trong tập chính (đặc biệt nếu có từ khóa), và không ghi âm hội thoại riêng tư của người khác.

**Ambient (8 phiên × 30–60 giây/collector):** nên ghi 1 phiên phòng yên tĩnh; 1 quạt; 1 điều hòa/PC; 1 tiếng hành lang/bước chân; 1 bàn phím/cửa/ghế; 1 tiếng động cơ/bánh xe robot; 1 hội thoại xa đã có đồng ý; 1 môi trường khác hoặc phòng có tiếng vang. Bấm `ambient` và chọn 30 hoặc 60 giây; **không đọc dòng hướng dẫn ambient trên GUI**. Ambient tuyệt đối không có từ gọi robot thật.

## 7. Cách đọc từ khóa và khoảng nghỉ giữa các lần ghi

**Trong 25 mẫu positive / speaker:**

- **10 mẫu chỉ nói “Sô ta”**; thay đổi tốc độ, khoảng cách, góc và âm lượng tự nhiên.
- **5 mẫu nói “Sô ta ơi”**.
- **10 mẫu câu gọi ngắn** như “Này Sô ta”, “Sô ta, lại đây”, “Sô ta, giúp mình với”, “Sô ta, nghe không?”.

GUI luân phiên các câu này trong 25 gợi ý đầu; bấm **“Câu tiếp theo”** nếu muốn đổi. Khi chọn speaker khác, ứng dụng căn vị trí câu gợi ý theo số WAV đã lưu của speaker đó (theo nhãn hiện tại). Không nhất thiết nói thật nhiều câu dài sau wakeword; mô hình phải nhận được từ gọi, không học thuộc cả câu lệnh.

**Tốc độ nói:** khoảng 60% bình thường, 20% chậm hơn tự nhiên, 20% nhanh hơn tự nhiên; không cần hát, hét hoặc phát âm một cách máy móc. **Âm lượng:** chủ yếu giọng nói thường; thỉnh thoảng nói nhỏ hơn hoặc nhấn giọng nhẹ, nhưng không được méo/clipping.

**Trong mỗi bản thu:**

1. Chọn đúng speaker/nhãn, kiểm tra câu gợi ý, đặt người nói ở vị trí đã chọn.
2. Nhấn **Bắt đầu ghi**. Chờ **0,3–0,5 giây** rồi nói **một lần** câu gợi ý.
3. Để **0,3–0,5 giây** yên lặng cuối câu trước khi kết thúc. Dùng **3 giây** cho lời gọi ngắn, **5 giây** nếu câu dài để tránh bị cắt.
4. **Nghe lại**; đúng nhãn/đủ âm thì **Lưu**, không đạt thì **Bỏ bản thu**.
5. Nghỉ khoảng **2–5 giây giữa hai lượt thu**, thay đổi khoảng cách/góc hoặc câu nói; đây là thời gian nghỉ **khuyến nghị để vận hành**, không phải yêu cầu kỹ thuật của thuật toán.
6. Sau khoảng 5–10 file, đổi khoảng cách/góc hoặc điều kiện nhiễu; tránh thu nguyên một nhãn duy nhất ở một vị trí.

## 8. Tiêu chí kiểm tra và loại bỏ mẫu

**Chấp nhận khi:** đúng nhãn; nghe rõ lời; cả hai âm “Sô” và “ta” có mặt và không bị cắt trong positive; không có wakeword trong nhóm còn lại; file WAV hợp lệ **16 kHz, mono, PCM signed 16-bit**; không bị méo tiếng, tiếng bật micro hay cọ xát mạnh.

**Loại và thu lại nếu:**

- Nhãn không đúng, đọc nhầm sang “sổ tay” hoặc ngược lại.
- Có người thứ hai nói wakeword thật trong mẫu negative/hard-negative/ambient.
- Micro bị chạm, bị rè/clipping, âm lượng quá bé hoặc gói âm bị mất.
- Thiếu đầu/cuối từ khóa; lời đọc bị cắt do bấm ngừng quá sớm.
- Có nhạc/TV/giọng không được phép thu hoặc thông tin riêng tư của người khác.
- Nhiễu che lấp hoàn toàn tiếng của người nói.

**Đặc biệt:** không dùng cách lấy cùng một WAV rồi tăng/giảm volume, đổi tên file hoặc phát lại qua loa để tính thành nhiều bản thu thực tế. Data augmentation nếu cần sẽ do nhóm huấn luyện thực hiện và quản lý riêng.

## 9. Checklist bàn giao ZIP

- [ ] Collector dùng đúng tên/mã riêng, ví dụ `c01`, `hoanganh` (2–12 ký tự không dấu; giữ nguyên qua các lần gửi).
- [ ] Đã thu ít nhất 3 speaker độc lập; mục tiêu 5 speaker; không dùng lại tên speaker cho người khác.
- [ ] Mỗi speaker có đủ positive, negative và hard_negative như bảng quota.
- [ ] Mỗi nhãn xuất hiện ở nhiều khoảng cách, góc và điều kiện tiếng nhiễu.
- [ ] Có đủ các phiên ambient 30–60 giây.
- [ ] Đã nghe kiểm tra mẫu ngẫu nhiên của từng người/nhãn, không còn nhãn sai hoặc file lỗi.
- [ ] Nhấn **Xuất ZIP dataset** trên GUI; ZIP có `dataset/raw/positive`, `negative`, `hard_negative`, `ambient`.
- [ ] Chỉ gửi ZIP qua kênh riêng tư cho người tổng hợp; **không gửi `recorder_registry.json`**.
- [ ] Gửi kèm thông tin: mã collector, số speaker, số WAV từng nhãn, số clip ambient, số địa điểm/thiết bị và ghi chú điều kiện đặc biệt.

## 10. Nền tảng kỹ thuật tham khảo

- Google Speech Commands — bộ dữ liệu KWS nhằm phân biệt từ kích hoạt với lời nói không liên quan và nhiễu nền: https://arxiv.org/abs/1804.03209 ; https://github.com/tensorflow/datasets/blob/master/docs/catalog/speech_commands.md
- Nghiên cứu về near-field/far-field keyword spotting và ảnh hưởng của các khoảng cách/tiếng TV/âm nền: https://link.springer.com/article/10.1186/s13636-023-00293-8
- ETSI TS 103 504 — phương pháp đánh giá thiết bị điều khiển bằng giọng nói ở khoảng cách xa (tham khảo thiết kế bài thử, không đồng nhất với quota thu thập ở tài liệu này): https://www.etsi.org/deliver/etsi_ts/103500_103599/103504/01.01.01_60/ts_103504v010101p.pdf

**Lưu ý:** các mức 0,4–2,0 m, góc ±30°/±60°, 333 file/collector và thời gian nghỉ 2–5 giây là **đề xuất kỹ thuật riêng của dự án** để có bộ dữ liệu đa dạng, không phải thông số được các tài liệu trên quy định.
