# Visitor UI

## Phạm vi phiên bản đầu

1. Dẫn đường.
2. Hỏi đáp thông tin.
3. Cài đặt công khai: ngôn ngữ, cỡ chữ, âm lượng, giới thiệu robot.
4. Feedback người dùng.

## Nguyên tắc

- Khách chỉ thấy giao diện đơn giản, không thấy ROS graph, log, TF, raw sensor hoặc điều khiển tay.
- Browser không publish trực tiếp ROS topic.
- Backend mới của visitor UI là lớp duy nhất được phép gửi `NavigateToPose` và cancel action.
- Không publish `/cmd_vel` từ visitor UI.
- Mọi lệnh điều hướng cần xác nhận trên màn hình, kiểm tra map và giới hạn vùng đi được.
- Q&A ưu tiên knowledge base local; không phụ thuộc Internet hoặc LLM ở bản đầu.

## Cấu trúc

```text
visitor_ui/
├── README.md
├── config/
│   ├── visitor_ui.yaml         # Theme, language, API, safety limits
│   ├── places.yaml             # Điểm đến/POI được phép dẫn đường
│   └── knowledge_base.vi.yaml  # Nội dung hỏi đáp tiếng Việt ban đầu
└── docs/
    ├── ARCHITECTURE.md
    └── IMPLEMENTATION_PLAN.md
```

Chưa tạo `package.xml`, `setup.py` hoặc frontend source. Điều này giữ `colcon build` hiện tại không đổi cho đến khi bắt đầu implementation.

## Tài liệu

- Kiến trúc UX, emotion, data source, ROS boundary: `docs/ARCHITECTURE.md`.
- Roadmap thực hiện theo phase: `docs/IMPLEMENTATION_PLAN.md`.
- Cấu hình/nội dung mẫu: `config/`.
