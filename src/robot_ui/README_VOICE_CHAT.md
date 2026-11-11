# Souta Robot Kiosk Multilingual Voice Chat

Tài liệu kỹ thuật chi tiết về hệ thống đàm thoại giọng nói đa ngôn ngữ (Vietnamese, English, Japanese, Chinese, Korean) tích hợp trực tiếp trên giao diện cảm ứng Souta Robot Kiosk.

---

## 1. Kiến trúc Pipeline (Audio & AI Pipeline)

```mermaid
flowchart TD
    User([Người dùng nói]) --> Mic[USB Microphone / parec]
    Mic --> VAD[RMS VAD & Silence Detector]
    VAD -->|Hợp lệ| STT[Groq Whisper V3 Turbo]
    VAD -->|Im lặng / Rác| Skip[Bỏ qua / Nhắc nói lại]
    
    STT --> Blacklist{Lọc Hallucination}
    Blacklist -->|Rác| Retry[Báo 'Không nghe rõ']
    Blacklist -->|Văn bản sạch| Bubble[Hiển thị Bubble Chat Người Dùng]
    
    Bubble --> LLM[9Router LLM - Combo hehe + RAG]
    LLM --> Answer[Hiển thị Bubble Chat Souta]
    
    Answer --> Router[resolve_voice_and_text]
    Router -->|Tiếng Việt| HoaiMy[Hoài My: Đổi Souta -> Sota]
    Router -->|Tiếng Anh| Jenny[Jenny: en-US]
    Router -->|Tiếng Nhật| Nanami[Nanami: ja-JP]
    Router -->|Tiếng Trung| Xiaoyi[Xiaoyi: zh-CN]
    Router -->|Tiếng Hàn| JiMin[Ji-Min: ko-KR]
    
    HoaiMy --> TTS[9Router Edge-TTS: /v1/audio/speech]
    Jenny --> TTS
    Nanami --> TTS
    Xiaoyi --> TTS
    JiMin --> TTS
    
    TTS --> Player[AudioPlayer: mpv subprocess stdin]
    Player --> Speaker([Loa phát âm thanh ra Kiosk])
```

---

## 2. Phân chia trách nhiệm các Module (`robot_ui/kiosk/`)

| File / Module | Trách nhiệm chính |
| :--- | :--- |
| `audio_capture.py` | Quản lý tiến trình `parec`, thu âm stream PCM 16kHz Mono `S16_LE`, tính toán RMS phát hiện tiếng nói, tự động ngắt khi phát hiện im lặng (>1.0s) hoặc đạt giới hạn tối đa (15s), đóng gói WAV trong bộ nhớ RAM. |
| `speech_client.py` | Gửi audio WAV lên Groq Whisper API, lọc hallucination blacklist; Phân tích nhận diện 5 ngôn ngữ, chuẩn hóa ngữ âm tiếng Việt (`Souta` $\rightarrow$ `Sota`), lọc markdown/URL và gọi 9Router Edge-TTS. |
| `audio_player.py` | Điều khiển tiến trình `mpv` phát audio trực tiếp qua stdin pipe; Quản lý PID cụ thể, ngắt tức thì khi bấm "Dừng nói" mà không dùng `pkill`; Hỗ trợ bật/tắt loa (mute/unmute) không để response cũ phát trễ. |
| `audio_worker.py` | `VoiceController` điều phối State Machine (`READY -> LISTENING -> STT -> LLM -> TTS -> SPEAKING -> READY`), quản lý `session_id` và `turn_id` nhằm loại bỏ triệt để các kết quả đến muộn sau khi reset phiên. |
| `widgets.py` | Giao diện `ChatPage`: Quản lý nút Mic (các trạng thái kèm hiệu ứng pulse đỏ/vàng khi ghi âm), nút Bật/Tắt loa, nút "Dừng nói", API `submit_voice_transcript()`. |
| `window.py` | Tích hợp `VoiceController` với Qt GUI, bảo vệ Inactivity Idle Timeout (không reset khi đang nghe/nói), tự động hủy tác vụ audio khi người dùng chuyển trang. |
| `llm_env.py` | Bộ nạp biến môi trường an toàn, hỗ trợ đọc các thiết lập Voice từ `.env` mà không thực thi shell nguy hiểm. |

---

## 3. Danh sách Thiết bị Microphone & Loa được hỗ trợ

* **Microphone đầu vào:**
  * Khuyên dùng: `Jieli Technology USB Composite Device (USB Audio)` (Microphone webcam USB).
  * PulseAudio Source: `alsa_input.usb-Jieli_Technology_USB_Composite_Device-02.mono-fallback` (hoặc `@DEFAULT_SOURCE@`).
* **Loa phát ra:**
  * Mặc định: PulseAudio / PipeWire default sink.
  * Engine phát: `mpv` (độ trễ thấp, không chiếm giữ file tạm lâu dài).

---

## 4. Bảng Giọng Nói Đa Ngôn Ngữ & Xử lý Chuẩn Hóa

| Ngôn ngữ | Model Voice TTS | Nhận diện ngôn ngữ | Chuẩn hóa đặc biệt |
| :--- | :--- | :--- | :--- |
| **Tiếng Việt** | `edge-tts/vi-VN-HoaiMyNeural` | Ký tự dấu thanh tiếng Việt | Tự động thay thế `Souta` $\rightarrow$ `Sota` để phát âm tự nhiên |
| **Tiếng Anh** | `edge-tts/en-US-JennyNeural` | Ký tự Latin / ASCII | Giọng US accent tự nhiên |
| **Tiếng Nhật** | `edge-tts/ja-JP-NanamiNeural` | Ký tự Hiragana / Katakana | Giọng Nanami trong trẻo |
| **Tiếng Trung** | `edge-tts/zh-CN-XiaoyiNeural` | Chữ Hán (`\u4e00-\u9fff`) không có Kana | Giọng Xiaoyi trẻ trung |
| **Tiếng Hàn** | `edge-tts/ko-KR-JiMinNeural` | Ký tự Hangul (`\uac00-\ud7af`) | Giọng Ji-Min dễ thương |

---

## 5. Bảng Trạng Thái Icon Giao diện

| Trạng thái | Icon Mic | Hiệu ứng / Trạng thái nút | Dòng trạng thái (Subtitle) |
| :--- | :--- | :--- | :--- |
| `READY` | `mic.png` | Nút kích hoạt, sẵn sàng chạm | Sẵn sàng trò chuyện |
| `LISTENING` | `mic_listening.png` | Pulse viền đỏ/vàng nhấp nháy liên tục | Đang nghe... Chạm lần nữa để gửi |
| `STT_PROCESSING` | `mic.png` | Nút khóa mờ, đang gọi Groq Whisper | Đang nhận dạng giọng nói... |
| `LLM_PROCESSING` | `mic.png` | Nút khóa mờ, LLM đang xử lý RAG | Thinking... |
| `TTS_PROCESSING` | `mic.png` | Hiện nút **Dừng nói** | Đang tạo giọng nói... |
| `SPEAKING` | `mic.png` | Hiện nút **Dừng nói**, mpv đang phát | Souta đang nói... |
| `UNAVAILABLE` | `mic_off.png` | Không tìm thấy thiết bị thu âm | Không tìm thấy microphone |
| `ERROR` | `mic_off.png` | Lỗi kết nối / thu âm (tự phục hồi sau 2.5s) | Lỗi microphone / Không thể kết nối STT |

---

## 6. Cấu hình Môi trường (Environment Variables)

File mẫu: `config/voice.example.env` (sao chép sang `config/voice.local.env` hoặc nhập vào `config/llm.local.env`):

```dotenv
# STT (Groq Whisper Large V3 Turbo)
GROQ_API_KEY=gsk_...
GROQ_STT_MODEL=whisper-large-v3-turbo
GROQ_STT_LANGUAGE=auto

# Audio Input
# Để trống để sử dụng PulseAudio Default Source (mic webcam)
KIOSK_MIC_SOURCE=
AUDIO_SAMPLE_RATE=16000
AUDIO_CHANNELS=1
AUDIO_MAX_RECORD_SECONDS=15
AUDIO_MIN_RECORD_SECONDS=0.3
AUDIO_SILENCE_TIMEOUT_SECONDS=1.0
AUDIO_RMS_THRESHOLD=350.0

# TTS (Edge TTS qua 9Router)
KIOSK_TTS_ENABLED=true
KIOSK_TTS_BASE_URL=http://127.0.0.1:20128/v1
KIOSK_TTS_DEFAULT_LANGUAGE=vi
KIOSK_TTS_AUTO_SPEAK=true

# Player
KIOSK_AUDIO_PLAYER=mpv
```

---

## 7. Hướng dẫn Biên dịch & Khởi chạy

### Bước 1: Cài đặt Dependencies hệ thống (nếu máy mới)
```bash
sudo apt update
sudo apt install -y pulseaudio-utils mpv python3-requests
```

### Bước 2: Build gói ROS 2
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_ui --symlink-install
source install/setup.bash
```

### Bước 3: Chạy Kiosk UI
```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=false
```

---

## 8. Hướng dẫn Kiểm thử (Testing)

### Chạy tự động toàn bộ Unit & GUI Tests (50 tests):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/robot_ui:$PYTHONPATH QT_QPA_PLATFORM=offscreen python3 -m pytest src/robot_ui/test/ -v
```

### Kiểm tra nhanh STT thủ công từ Terminal:
```bash
timeout 4 parecord --channels=1 --rate=16000 /tmp/test_mic.wav && mpv /tmp/test_mic.wav
```

### Kiểm tra nhanh TTS 5 thứ tiếng từ Terminal:
```bash
# Tiếng Việt:
curl -s -X POST http://127.0.0.1:20128/v1/audio/speech \
  -H "Authorization: Bearer sk-3cbf1313d8e08dd5-y4kybe-c7fa045d" \
  -H "Content-Type: application/json" \
  -d '{"model": "edge-tts/vi-VN-HoaiMyNeural", "input": "Xin chào, tôi là Sota Robot."}' | mpv -
```

---

## 9. Troubleshooting & Xử lý Sự cố

1. **Microphone không thu được tiếng (Mic rỗng / âm lượng nhỏ):**
   * Chạy `pactl list sources short` để xem thiết bị mic USB có đang ở trạng thái `RUNNING` hoặc `IDLE` không.
   * Chạy `pactl set-source-volume @DEFAULT_SOURCE@ 95%` để tăng âm lượng thu âm.
2. **Whisper báo lỗi `401 Unauthorized` hoặc `Rate limit 429`:**
   * Kiểm tra biến `GROQ_API_KEY` trong `config/llm.local.env` hoặc `~/.9router/db/data.sqlite`.
3. **9Router TTS báo lỗi 502 hoặc không phát âm thanh:**
   * Đảm bảo tiến trình 9Router đang chạy trên port `20128` (`curl http://127.0.0.1:20128/v1/models`).
   * Tên model TTS bắt buộc phải có tiền tố `edge-tts/` (ví dụ `edge-tts/vi-VN-HoaiMyNeural`).
4. **Không dừng được âm thanh khi người dùng nhấn "Dừng nói":**
   * Lớp `AudioPlayer` quản lý trực tiếp PID của `mpv`. Tiến trình sẽ nhận tín hiệu `SIGTERM` và giải phóng cổng audio ngay lập tức trong vòng < 0.3s.
