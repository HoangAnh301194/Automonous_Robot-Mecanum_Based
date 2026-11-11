# Tài liệu Kỹ thuật Bộ Video Biểu cảm (Emotion Videos) — Souta Robot

## 1. Giới thiệu tổng quan

Bộ biểu cảm của Souta Robot được xây dựng dưới dạng các video ngắn định dạng MP4 hiển thị trên màn hình chờ chính (`HomePage` / `idlePage`) của giao diện Robot UI Kiosk (PyQt5, ROS 2 Humble).

### Mục đích sử dụng
- **Nhận diện thương hiệu & tạo thiện cảm:** Đóng vai trò là khuôn mặt đại diện của Souta Robot khi giao tiếp với người dùng và khách hàng tại Học viện Công nghệ Bưu chính Viễn thông (PTIT).
- **Phản hồi trạng thái trực quan:** Giúp người xung quanh nắm bắt tức thì tình trạng hoạt động của robot (đang chờ, đang di chuyển, sắp hết pin/nước, cần cảnh báo, hoặc gặp sự cố) mà không cần nhìn vào bảng điều khiển chi tiết.

### Cơ chế phát video trong Kiosk
- **Nền tảng:** Sử dụng `PyQt5` kết hợp `OpenCV` (`cv2.VideoCapture`).
- **Lớp hiển thị:** Lớp `VideoBackground` trong [`src/robot_ui/robot_ui/kiosk/widgets.py`](file:///media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based/src/robot_ui/robot_ui/kiosk/widgets.py#L39-L130).
  - Một `QTimer` được khởi tạo với chu kỳ tính theo FPS của video (`round(1000.0 / fps)`).
  - Tại mỗi chu kỳ, frame hình ảnh được giải mã qua OpenCV, chuyển hệ màu sang `QImage.Format_RGB888`, chuyển thành `QPixmap` và vẽ trực tiếp lên widget nền thông qua `QPainter`.
  - Khung hình được căn giữa và co dãn theo tỷ lệ `display_scale` (mặc định `0.9`) và dịch trục dọc `vertical_offset_px` (mặc định `-30 px`).
- **Cơ chế nạp cấu hình:** Lớp `EmotionCatalog` trong [`src/robot_ui/robot_ui/kiosk/assets.py`](file:///media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based/src/robot_ui/robot_ui/kiosk/assets.py#L22-L37) đọc cấu hình từ file [`config/emotions.json`](file:///media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based/src/robot_ui/config/emotions.json), chuyển đổi tên logic của cảm xúc thành đường dẫn file video MP4 tuyệt đối trên đĩa.

---

## 2. Bảng thông số kỹ thuật (Đo đạc trực tiếp từ 10 file MP4)

Toàn bộ 10 video MP4 hiện có đều được đo kiểm trực tiếp thông qua thư viện OpenCV và ffprobe:

| STT | Tên file | Độ phân giải | Tốc độ khung hình (FPS) | Tổng số frame | Thời lượng | FourCC / Codec | Dung lượng | Sai lệch vòng lặp (L1 Diff) |
|---|---|---|---|---|---|---|---|---|
| 1 | `normal.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 294 KB | 0.23 px |
| 2 | `angry.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 367 KB | 0.24 px |
| 3 | `laught.mp4` *(legacy)* | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 683 KB | 0.21 px |
| 4 | `omoshiroi.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 862 KB | 0.22 px |
| 5 | `sleepy.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 363 KB | 0.30 px |
| 6 | `sleeping.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 322 KB | 0.26 px |
| 7 | `cell_0_1.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 239 KB | 0.24 px |
| 8 | `cell_2_1.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 395 KB | 0.22 px |
| 9 | `cell_3_0.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 379 KB | 0.48 px |
| 10 | `cell_4_0.mp4` | 360 × 234 px | 24.00 fps | 401 frames | 16.71 s | FMP4 (MPEG-4 Part 2) | 505 KB | 0.23 px |

> **Nhận xét kỹ thuật:**
> - Tất cả 10 video có sự chuẩn hóa hình học và chu kỳ thời gian đồng nhất tuyệt đối (độ phân giải 360×234, 24 fps, 401 frames, 16.71 giây).
> - Sai lệch L1 giữa khung hình đầu tiên (frame 0) và khung hình cuối cùng (frame 400) cực kỳ nhỏ (< 0.5 pixel), bảo đảm chuyển tiếp vòng lặp liên tục (seamless loop) không bị giật hay nhấp nháy đen.

---

## 3. Danh mục chi tiết 10 Video (Video Catalog)

### 1. `normal.mp4`
- **Tên mô tả tiếng Anh:** Normal / Idle Face
- **Tên cảm xúc tiếng Việt:** Biểu cảm mặc định
- **Hình dạng mắt:** Hai mắt màu trắng dạng viên thuốc bo tròn (capsule-shaped, kích thước ~74×106 px trên nền đen).
- **Hình dạng miệng:** Thanh ngang bo góc mảnh nhỏ (~66×25 px).
- **Chuyển động chính:** Mắt chớp đều đặn mỗi 2 đến 3 giây (thu hẹp thành dải sáng ngang ~97×15 px rồi mở to trở lại).
- **Hiệu ứng đặc biệt:** Không có.
- **Trạng thái sử dụng phù hợp:** Chế độ chờ (`IDLE`), robot sẵn sàng tiếp nhận tương tác.
- **Trạng thái không nên sử dụng:** Khi robot có sự cố hoặc cần biểu đạt cảm xúc chuyên biệt (vui mừng, cảnh báo, lỗi).
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) hoàn hảo.

### 2. `angry.mp4`
- **Tên mô tả tiếng Anh:** Angry / Frustrated Face
- **Tên cảm xúc tiếng Việt:** Tức giận / Cảnh báo
- **Hình dạng mắt:** Mắt xếch góc nhọn hướng vào trong tạo thế nhíu mày dữ dội (`\ /`).
- **Hình dạng miệng:** Nhếch xuống gắt gỏng hoặc biến đổi nhọn.
- **Chuyển động chính:** Giật rung nhẹ toàn bộ khuôn mặt biểu thị sự khó chịu/tức giận.
- **Hiệu ứng đặc biệt:** Đốm xung biểu thị dấu hiệu gân giận dữ ở khóe trán.
- **Trạng thái sử dụng phù hợp:** Cảnh báo vật cản bất ngờ chặn đầu robot, pin hoặc nước ở mức cảnh báo (`warning`).
- **Trạng thái không nên sử dụng:** Trạng thái chào khách hoặc giao tiếp dịch vụ thông thường.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 3. `laught.mp4` *(Lưu ý: Tên legacy có lỗi chính tả, giữ nguyên để đảm bảo tương thích mã nguồn)*
- **Tên mô tả tiếng Anh:** Laughing / Happy Face
- **Tên cảm xúc tiếng Việt:** Cười tươi / Vui vẻ
- **Hình dạng mắt:** Mắt híp lại thành hình vòng cung cong hướng lên (`^ ^`).
- **Hình dạng miệng:** Há to hình miệng cười chữ U rộng sảng khoái.
- **Chuyển động chính:** Cả khuôn mặt nhấp nhô theo nhịp cười vui nhộn.
- **Hiệu ứng đặc biệt:** Không có.
- **Trạng thái sử dụng phù hợp:** Hoàn thành nhiệm vụ phục vụ nước (`happy`), chào tạm biệt khách, câu trả lời thân thiện từ Chat LLM.
- **Trạng thái không nên sử dụng:** Khi có sự cố, cảnh báo nguy hiểm hoặc đang di chuyển nhanh.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 4. `omoshiroi.mp4`
- **Tên mô tả tiếng Anh:** Excited / Curious / Sparkle Face
- **Tên cảm xúc tiếng Việt:** Hào hứng / Tò mò / Thú vị
- **Hình dạng mắt:** Mắt mở to tròn, điểm xuyết hạt màu xanh lơ và vàng nhạt.
- **Hình dạng miệng:** Miệng cười mỉm nhỏ.
- **Chuyển động chính:** Mắt mở to tò mò, có nhướn mày thể hiện sự chú ý cao độ.
- **Hiệu ứng đặc biệt:** Các hạt lấp lánh (sparkles) tỏa sáng xung quanh vùng mắt và trán.
- **Trạng thái sử dụng phù hợp:** Lắng nghe người dùng nói (`listening`), phát hiện người bước tới trước mặt (`detection`).
- **Trạng thái không nên sử dụng:** Báo lỗi hệ thống, cảnh báo cạn pin/nước.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 5. `sleepy.mp4`
- **Tên mô tả tiếng Anh:** Drowsy / Sleepy Face
- **Tên cảm xúc tiếng Việt:** Buồn ngủ / Mệt mỏi
- **Hình dạng mắt:** Mí mắt trên sụp xuống một nửa, mắt lờ đờ chớp chậm rãi.
- **Hình dạng miệng:** Miệng há to ngáp dài theo chu kỳ.
- **Chuyển động chính:** Đầu gật gù buồn ngủ, ngáp chậm rãi.
- **Hiệu ứng đặc biệt:** Vệt mệt mỏi dưới mi mắt.
- **Trạng thái sử dụng phù hợp:** Chế độ pin sắp cạn (chuẩn bị quay về dock sạc), chuẩn bị chuyển sang Sleep mode.
- **Trạng thái không nên sử dụng:** Tuyệt đối không dùng cho trạng thái `thinking` (suy luận LLM/RAG) vì gây hiểu lầm robot bị đơ hoặc chán nản.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 6. `sleeping.mp4`
- **Tên mô tả tiếng Anh:** Sleeping / Standby Face
- **Tên cảm xúc tiếng Việt:** Đang ngủ / Chế độ nghỉ
- **Hình dạng mắt:** Nhắm nghiền thành hai đường cong mảnh nằm ngang (`- -`).
- **Hình dạng miệng:** Miệng nhỏ khép hờ.
- **Chuyển động chính:** Nhịp thở nhẹ nhàng (co giãn nhẹ khuôn mặt theo chu kỳ thở).
- **Hiệu ứng đặc biệt:** Ký tự bong bóng "Zzz" màu xám-trắng bay lên từ góc trên bên phải.
- **Trạng thái sử dụng phù hợp:** Chế độ Standby qua đêm, cắm sạc pin, chế độ tiết kiệm năng lượng.
- **Trạng thái không nên sử dụng:** Khi đang phục vụ hoặc đang lăn bánh di chuyển.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 7. `cell_0_1.mp4`
- **Tên mô tả tiếng Anh:** Face Morphing / Transition Animation
- **Tên cảm xúc tiếng Việt:** Chuyển đổi hình học / Khởi động
- **Hình dạng mắt:** Biến đổi từ các sọc đứng hẹp sang mắt viên thuốc tiêu chuẩn.
- **Hình dạng miệng:** Xuất hiện và co dãn theo nhịp biến đổi.
- **Chuyển động chính:** Biến hình kỹ thuật (morphing animation).
- **Hiệu ứng đặc biệt:** Kỹ xảo chuyển cảnh hình học.
- **Trạng thái sử dụng phù hợp:** Hiệu ứng lúc boot hệ thống, thức dậy sau khi sleep.
- **Trạng thái không nên sử dụng:** Không phù hợp làm biểu cảm `welcome` kéo dài liên tục.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 8. `cell_2_1.mp4`
- **Tên mô tả tiếng Anh:** Speedometer / Tachometer Gauge
- **Tên cảm xúc tiếng Việt:** Đồng hồ tốc độ điều hướng
- **Hình dạng mắt & mặt:** Không có mắt người, thay thế bằng cụm đồng hồ đo tốc độ hình tròn.
- **Hình dạng miệng:** Vạch chỉ số trung tâm.
- **Chuyển động chính:** Kim đồng hồ quét và các vạch chia tốc độ sáng tắt tuần tự.
- **Hiệu ứng đặc biệt:** Màu cam-đỏ nổi bật trên nền đen.
- **Trạng thái sử dụng phù hợp:** Trạng thái đang di chuyển Nav2 (`MOVING`, `FETCHING_LOCATION`).
- **Trạng thái không nên sử dụng:** Khi robot đứng yên giao tiếp trực diện với con người.
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 9. `cell_3_0.mp4`
- **Tên mô tả tiếng Anh:** Eye Scanning / Observing Radar
- **Tên cảm xúc tiếng Việt:** Quan sát / Quét không gian
- **Hình dạng mắt:** Mắt mở to, con ngươi và quầng sáng dịch chuyển liên tục từ trái sang phải.
- **Hình dạng miệng:** Miệng nhỏ trung tính.
- **Chuyển động chính:** Đảo mắt quan sát môi trường theo góc rộng.
- **Hiệu ứng đặc biệt:** Vệt sáng quét ngang dạng radar.
- **Trạng thái sử dụng phù hợp:** SLAM mapping, tìm kiếm mục tiêu, quét tìm người (`searching`).
- **Trạng thái không nên sử dụng:** Không tối ưu cho trạng thái `arrived` (đến nơi cần chào mừng hơn là quét tìm).
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

### 10. `cell_4_0.mp4`
- **Tên mô tả tiếng Anh:** Heart Eyes / In Love / Affection
- **Tên cảm xúc tiếng Việt:** Mắt trái tim / Yêu thích
- **Hình dạng mắt:** Mắt hình trái tim hoặc mắt sáng to với trái tim nổi bật.
- **Hình dạng miệng:** Miệng cười duyên dáng.
- **Chuyển động chính:** Nhấp nháy trái tim nhịp nhàng.
- **Hiệu ứng đặc biệt:** Các trái tim màu hồng-tím bay lên quanh gò má.
- **Trạng thái sử dụng phù hợp:** Nhận được lời khen từ người dùng, biểu cảm cảm ơn, sự kiện đặc biệt.
- **Trạng thái không nên sử dụng:** **Tuyệt đối không dùng cho lỗi hệ thống (`error`)** vì sai lệch nghiêm trọng về mặt ngữ nghĩa!
- **Ghi chú về loop:** Lặp liên tục (`LOOP`) mượt.

---

## 4. Ánh xạ Cảm xúc hiện tại (`config/emotions.json`)

Nội dung cấu hình đang hoạt động trong hệ thống:

```json
{
  "default": "idle",
  "display_scale": 0.9,
  "vertical_offset_px": -30,
  "emotions": {
    "idle": "normal.mp4",
    "welcome": "cell_0_1.mp4",
    "happy": "laught.mp4",
    "listening": "omoshiroi.mp4",
    "thinking": "sleepy.mp4",
    "navigating": "cell_2_1.mp4",
    "arrived": "cell_3_0.mp4",
    "warning": "angry.mp4",
    "error": "cell_4_0.mp4",
    "sleeping": "sleeping.mp4"
  }
}
```

### Bảng phân tích chi tiết Ánh xạ (Mapping Review):

| Emotion Key | File video cấu hình | Trạng thái trong runtime code | Đánh giá tính phù hợp | Kiến nghị điều chỉnh tương lai |
|---|---|---|---|---|
| `idle` | `normal.mp4` | **Đang sử dụng** | **Rất phù hợp** | Giữ nguyên. |
| `welcome` | `cell_0_1.mp4` | *Chưa được gọi* | **Chưa phù hợp** (kỹ xảo morphing) | Tạo mới `welcome_v2.mp4` (mắt chớp tươi cười vẫy chào). |
| `happy` | `laught.mp4` | *Chưa được gọi* | **Rất phù hợp** | Kết nối vào code khi khách bấm "Hoàn tất phục vụ". |
| `listening` | `omoshiroi.mp4` | *Chưa được gọi* | **Tạm chấp nhận** | Tương lai tạo `listening_v2.mp4` (mắt tập trung + sóng âm). |
| `thinking` | `sleepy.mp4` | *Chưa được gọi* | **Không phù hợp** (buồn ngủ/ngáp) | Tạo mới `thinking_v2.mp4` (mắt nhìn lên ngẫm nghĩ + loading dots). |
| `navigating` | `cell_2_1.mp4` | **Đang sử dụng** | **Phù hợp** (đồng hồ tốc độ) | Giữ nguyên. |
| `arrived` | `cell_3_0.mp4` | **Đang sử dụng** | **Cần đánh giá** (quét radar tìm kiếm) | Thay thế bằng `arrived_v2.mp4` (nháy mắt tươi cười chào khách). |
| `warning` | `angry.mp4` | **Đang sử dụng** | **Tạm giữ** (hơi tiêu cực) | Chuyển sang biểu cảm lo lắng (`worried`) hoặc biểu tượng chấm than tam giác vàng. |
| `error` | `cell_4_0.mp4` | **Đang sử dụng** | **Sai lệch ngữ nghĩa nghiêm trọng (mắt tim)** | **Cấp bách thay bằng `error_v2.mp4` (mắt dấu X `x x` hoặc xoay vòng).** |
| `sleeping` | `sleeping.mp4` | *Chưa được gọi* | **Rất phù hợp** | Kết nối vào chế độ Standby tiết kiệm pin. |

---

## 5. Phân tích logic chọn biểu cảm hệ thống (`window.py`)

Hàm [`_update_emotion()`](file:///media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based/src/robot_ui/robot_ui/kiosk/window.py#L752-L772) trong `KioskWindow` thực hiện ánh xạ từ trạng thái phần cứng và điều hướng ROS 2 sang Emotion Key:

$$\text{RobotStatus / NavigationStatus} \longrightarrow \text{Emotion Key} \longrightarrow \text{Video MP4}$$

### Bảng điều kiện kích hoạt & Thứ tự ưu tiên thực tế:

| Thứ tự ưu tiên | Điều kiện logic kích hoạt | Emotion Key | Video MP4 | Ý nghĩa hiển thị |
|---|---|---|---|---|
| **1 (Cao nhất)** | `robot_state in {"ERROR", "FAILED"}` | `error` | `cell_4_0.mp4` | Robot gặp sự cố nghiêm trọng |
| **2** | `battery_low` hoặc `water_low` hoặc `not sensors_ok` (lidar/odom/loc lỗi) | `warning` | `angry.mp4` | Cảnh báo pin/nước/cảm biến |
| **3** | `nav_state == "REACHED_GOAL"` | `arrived` | `cell_3_0.mp4` | Robot đã di chuyển đến đích |
| **4** | `nav_state in {"FETCHING_LOCATION", "WAITING_FOR_NAV2", "MOVING"}` | `navigating` | `cell_2_1.mp4` | Robot đang điều hướng di chuyển |
| **5 (Thấp nhất)** | Các trường hợp còn lại (mặc định) | `idle` | `normal.mp4` | Chế độ chờ bình thường |

### Các vấn đề cần lưu ý trong vận hành:
1. **Phạm vi hiển thị:** `_update_emotion()` có guard condition `if self._active_page != "idle": return`. Biểu cảm chỉ hiển thị khi màn hình ở trang chờ `idle`. Khi người dùng chuyển sang `home`, `chat`, `service`, `admin`, hoặc `navigation`, video nền sẽ tạm dừng (`pause()`).
2. **Xung đột ưu tiên Warning đè Navigation:** Nếu robot bị pin thấp (`battery_low = True`) hoặc cạn nước (`water_low = True`), khi robot di chuyển (`MOVING`), video vẫn hiển thị `warning` (`angry.mp4`) thay vì `navigating` (`cell_2_1.mp4`), do `warning` có độ ưu tiên cao hơn `navigating`.
3. **Hiện tượng lặp vô tận trạng thái Arrived:** Khi robot đến đích (`REACHED_GOAL`), `_update_emotion()` đặt video là `cell_3_0.mp4`. Do cơ chế loop vô hạn của `VideoBackground`, video này sẽ lặp liên tục mãi cho đến khi có trạng thái mới, thay vì phát 1 lần rồi trở về `idle`.
4. **5 Emotion chưa được kích hoạt:** `welcome`, `happy`, `listening`, `thinking`, `sleeping` mới chỉ được định nghĩa trong JSON nhưng chưa có bất kỳ sự kiện ROS 2 hay UI nào gọi tới.

---

## 6. Quy ước đặt tên cho Animation mới

Để mở rộng bộ animation trong tương lai mà không làm hỏng tính tương thích hệ thống:
1. **Quy chuẩn tên file:**
   - Toàn bộ bằng chữ thường (`lowercase`).
   - Sử dụng định dạng `snake_case` (ví dụ: `thinking_process.mp4`).
   - Định dạng bắt buộc luôn là `.mp4`.
   - Có thể dùng hậu tố phiên bản `_v2`, `_v3` cho các motion nâng cấp thử nghiệm (ví dụ: `error_v2.mp4`, `welcome_v2.mp4`).
2. **Quy tắc bảo tồn:**
   - **Tuyệt đối không đổi tên 10 file gốc** (kể cả file lỗi chính tả `laught.mp4` để bảo đảm tương thích ngược với toàn bộ code hiện hành).
   - Tên key trạng thái logic trong JSON tách biệt hoàn toàn với tên file video thực tế trên đĩa.

---

## 7. Quy ước sử dụng Animation: LOOP vs ONE_SHOT

- **`LOOP` (Lặp liên tục):**
  - Dành cho các trạng thái kéo dài theo thời gian thực: `idle`, `navigating`, `sleeping`, `warning`, `error`.
  - Khung hình kết thúc phải khớp mịn với khung hình mở đầu (loop diff < 1.0 px).
- **`ONE_SHOT` (Phát 1 lần):**
  - Dành cho các sự kiện tức thời: `welcome` (chào 1 lượt khi mở máy), `arrived` (đến nơi nháy mắt chào khách 1 lượt rồi tự động về idle), `happy` (cười tươi cảm ơn rồi về bình thường).
  - *Lưu ý kỹ thuật:* Hiện tại `VideoBackground` trong kiosk tự động tua lại frame 0 (`CAP_PROP_POS_FRAMES = 0`) khi hết video. Cơ chế `ONE_SHOT` là quy ước kiến trúc đề xuất cho tương lai khi hệ thống bổ sung tín hiệu kết thúc video (`video_finished`).

---

## 8. Hướng dẫn thêm Animation mới vào hệ thống

Quy trình chuẩn gồm 7 bước:

1. **Chuẩn bị video:**
   - Định dạng: MP4, codec video H.264 / MPEG-4 Part 2.
   - Độ phân giải khuyến nghị: `360 × 234 px` (tỷ lệ chuẩn).
   - Tốc độ khung hình: `24.00 fps` hoặc `30.00 fps`.
   - Chu kỳ lặp mượt mà nếu là video loại `LOOP`.
2. **Đặt file vào thư mục:**
   - Sao chép file MP4 vào `src/robot_ui/souta_robot/videomp4/`.
3. **Cập nhật cấu hình ánh xạ:**
   - Mở file `src/robot_ui/config/emotions.json`.
   - Đăng ký key cảm xúc mới hoặc trỏ key cũ sang file mới trong mục `"emotions"`.
4. **Kiểm tra `EmotionCatalog`:**
   - Bảo đảm `catalog.has("key_mới")` trả về `True` và `catalog.path("key_mới")` trỏ đúng đường dẫn.
5. **Cập nhật logic kích hoạt:**
   - Mở `src/robot_ui/robot_ui/kiosk/window.py`, bổ sung điều kiện chuyển trạng thái trong hàm `_update_emotion()`.
6. **Kiểm tra & Build:**
   ```bash
   colcon build --packages-select robot_ui
   source install/setup.bash
   ```
7. **Chạy kiểm thử:**
   ```bash
   QT_QPA_PLATFORM=offscreen python3 -m pytest src/robot_ui/test/test_kiosk_interface.py -q
   ```

---

## 9. Đề xuất Mở rộng Emotion System trong Tương lai (Future Emotion System)

Danh mục 12 trạng thái hoàn chỉnh đề xuất cho các phiên bản tiếp theo của Souta Robot:

| Trạng thái (State Key) | Sự kiện kích hoạt dự kiến | Animation đề xuất | Chế độ | Điều kiện chuyển về IDLE | Mức ưu tiên |
|---|---|---|---|---|---|
| `ERROR` | ROS 2 robot_state = ERROR / FAILED, va chạm khẩn cấp | `error_v2.mp4` (Mắt dấu X `x x` xoay vòng) | LOOP | Lỗi được xóa (Recovery) | 1 (Cao nhất) |
| `WARNING` | Pin < 20%, cạn nước, lỗi cảm biến Lidar / Odom | `warning_v2.mp4` (Mắt lo lắng / Chấm than vàng) | LOOP | Pin/nước/cảm biến bình thường trở lại | 2 |
| `WAITING_SERVICE` | Robot đến điểm phục vụ, chờ khách lấy nước | `waiting_v2.mp4` (Mắt nhìn khay nước, mỉm cười) | LOOP | Khách chạm "Hoàn tất & Tiếp tục" | 3 |
| `ARRIVED` | Nav2 status = REACHED_GOAL | `arrived_v2.mp4` (Nháy mắt tươi cười chào khách) | ONE_SHOT | Phát hết 1 chu kỳ tự chuyển về IDLE | 4 |
| `NAVIGATING` | Nav2 status = MOVING | `cell_2_1.mp4` (Đồng hồ tốc độ) | LOOP | Robot dừng hoặc đến đích | 5 |
| `SPEAKING` | TTS đang phát âm thanh giọng nói qua loa | `speaking_v2.mp4` (Khuôn miệng mấp máy đồng bộ âm) | LOOP | TTS nói xong | 6 |
| `LISTENING` | Microphone đang mở ghi âm (User đang nói) | `listening_v2.mp4` (Mắt tập trung + Sóng âm thanh) | LOOP | Dứt câu nói (Voice end) | 7 |
| `THINKING` | LLM đang suy luận / RAG processing | `thinking_v2.mp4` (Mắt nhìn lên + 3 chấm loading) | LOOP | LLM sinh câu trả lời xong | 8 |
| `HAPPY` | Phục vụ nước thành công, nhận khen ngợi | `laught.mp4` (Mắt cười `^ ^`) | ONE_SHOT | Hết animation tự chuyển về IDLE | 9 |
| `WELCOME` | Khởi động Kiosk, phát hiện người bước tới gần | `welcome_v2.mp4` (Mắt sáng chào đón) | ONE_SHOT | Hết animation tự chuyển về IDLE | 10 |
| `SLEEPING` | Standby không tương tác > 15 phút, sạc pin qua đêm | `sleeping.mp4` (Mắt nhắm + Bong bóng Zzz) | LOOP | Có chạm màn hình hoặc lệnh mới | 11 |
| `IDLE` | Trạng thái sẵn sàng bình thường | `normal.mp4` (Mặt capsule chớp mắt tự nhiên) | LOOP | Mặc định khi không có sự kiện khác | 12 (Thấp nhất) |
