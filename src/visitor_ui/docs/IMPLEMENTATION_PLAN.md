# Visitor UI Implementation Plan

## Phạm vi MVP

- Dẫn đường bằng POI hoặc chạm map + kéo hướng.
- Hỏi đáp offline/local.
- Cài đặt public: language, accessibility, information.
- Feedback local.
- Emotion face làm màn hình idle và trạng thái chuyến đi.

Không thuộc MVP:

- Manual teleoperation.
- Map editor.
- ROS graph/log/hardware dashboard.
- LLM Internet/cloud.
- Person following hoặc điều hướng từ hand-wave.

## Phase 0 — Chuẩn bị runtime

Mục tiêu: xác nhận Nav2 và map đủ điều kiện trước khi viết command UI.

1. Chạy một bringup odometry duy nhất.
2. Chạy localization/Nav2; xác nhận `/map`, `/odom`, TF `map -> base_footprint`.
3. Gửi thử một `NavigateToPose` từ RViz.
4. Xác nhận action status, cancel và failure case.
5. Điền pose thực cho `config/places.yaml`.

Hoàn thành khi:

- Robot đến được một POI từ RViz.
- `/map` và robot pose khớp trên cùng frame `map`.
- Không có publisher TF/odom trùng.

## Phase 1 — UX prototype không ROS

Mục tiêu: duyệt luồng kiosk với dữ liệu giả trước.

Tạo `src/visitor_ui/frontend/`:

- React + TypeScript + Vite.
- Home emotion face.
- Bottom menu bốn chức năng.
- Navigation map mock, drag heading, confirmation modal.
- Q&A keyword mock.
- Language/accessibility settings.
- Feedback form + thank-you screen.
- Auto reset session sau idle timeout.

Hoàn thành khi:

- Dùng chuột/touch hoàn tất toàn bộ flow không cần ROS.
- Nút touch tối thiểu `72 px`.
- Việt/English đổi được không reload page.
- UI chạy full-screen ở độ phân giải màn hình robot.

## Phase 2 — Emotion component

Mục tiêu: tạo thành phần dùng lại trên mọi screen.

- `EmotionFace` vẽ SVG/CSS.
- Implement `idle`, `greeting`, `thinking`, `navigating`, `arrived`, `warning`, `offline`.
- State machine từ app event; chưa nối camera/YOLO.
- Motion reduce mode theo accessibility setting.

Hoàn thành khi:

- Mỗi state có snapshot/visual test.
- Không làm giảm FPS map animation.
- `navigating` thu nhỏ face thành header thay vì che map.

## Phase 3 — Backend và ROS bridge read path

Mục tiêu: lấy state thật nhưng chưa gửi command.

Tạo package runtime riêng, ví dụ:

```text
src/visitor_ui/runtime/
├── visitor_ui_server.py
├── visitor_ui_bridge.py
├── state_store.py
├── navigation_validation.py
└── content_repository.py
```

Implement:

- FastAPI REST + WebSocket.
- Subscribe `/map`, `/odom`, Nav2 status, `/battery`.
- TF lookup `map -> base_footprint`.
- Normalize payload cho frontend.
- Cache/downsample map, không stream ROS message raw sang browser.

Hoàn thành khi:

- Frontend hiển thị map thật, pose robot, availability Nav2.
- Mất ROS chuyển face sang `offline`.
- Không expose ros graph/log/camera trong API visitor.

## Phase 4 — Navigation command an toàn

Mục tiêu: map touch điều khiển `NavigateToPose`, không publish `/cmd_vel`.

Implement:

- `POST /navigation/preview`.
- Pixel/map coordinate conversion.
- Occupancy check, boundary check, obstacle clearance, minimum distance.
- Approved POI lookup.
- Confirmation token ngắn hạn.
- `NavigateToPose` action client.
- Cancel chỉ cho goal do visitor UI tạo.
- WebSocket progress/result/error.

Hoàn thành khi:

- Goal hợp lệ chạy được từ touchscreen.
- Goal occupied/unknown bị từ chối trước action.
- Chỉ có một visitor goal active.
- Cancel/timeout đưa UI về Home an toàn.

## Phase 5 — Content và hỏi đáp

Mục tiêu: trả lời thông tin đúng, dễ cập nhật, offline.

- Parse `knowledge_base.vi.yaml`.
- Normalize Vietnamese text, keyword/intent scoring.
- Category quick actions.
- Link câu trả lời với `place_id` để mở navigation confirmation.
- Thêm English content pack.
- Thêm schema validation cho YAML.

Hoàn thành khi:

- Các câu hỏi chuẩn có câu trả lời xác định.
- Có fallback: “Tôi chưa có thông tin này. Vui lòng liên hệ lễ tân.”
- Content sửa không cần sửa frontend source.

## Phase 6 — Feedback và quan sát vận hành

Mục tiêu: thu phản hồi không làm lộ dữ liệu cá nhân.

- SQLite migration/schema.
- Validate rating/category/comment.
- Retention cleanup `90 ngày`.
- Export CSV chỉ cho admin tool sau này.
- Đếm KPI: số session, goal started/completed/cancelled, top Q&A intents, rating.

Hoàn thành khi:

- Feedback vẫn lưu khi ROS offline.
- Không lưu camera/image/identity/raw question history.

## Phase 7 — Hardening trước triển khai sảnh

1. Kiosk mode: browser auto-start, disable browser chrome, auto recovery.
2. Network: backend chỉ bind localhost hoặc private LAN theo deploy.
3. API: request-size limit, rate limit, CORS allowlist.
4. Navigation: goal timeout, recovery text, cancel UI.
5. Accessibility: high contrast, font scale, motion reduce, bilingual review.
6. Safety: no `/cmd_vel`; no admin endpoint; Nav2/collision monitor vẫn là safety authority.
7. Privacy: privacy screen, content policy, feedback retention.
8. Soak test: idle reset, ROS restart, Nav2 unavailable, camera unavailable, Wi-Fi loss.

## Thứ tự code đề xuất

1. Frontend mock: home + emotion + menu.
2. Navigation map interaction mock.
3. Visitor API + ROS read bridge.
4. Nav2 preview/validation/action bridge.
5. Q&A local content.
6. Feedback SQLite.
7. Kiosk deployment và test robot thật.

## Quy tắc không chạm `robot_ui`

- Không sửa file trong `src/robot_ui/`.
- Không import Python module từ `robot_ui`.
- Không dùng chung database/process/service name.
- Chỉ dùng chung ROS topics/action công khai qua interface ROS 2.
- Deployment service sẽ có tên riêng, ví dụ `visitor-ui.service`.
