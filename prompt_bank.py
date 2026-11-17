"""Built-in reading prompts for SOUTA wakeword dataset collection.

Positive samples contain the exact invocation "Sô ta".
Hard negatives imitate its sound without speaking the invocation.
Prompts are instructions only; they are not written into the exported WAV ZIP.
"""
from __future__ import annotations

PROMPTS = {
    "positive": (
        "Sô ta",
        "Sô ta ơi",
        "Này Sô ta",
        "Chào Sô ta",
        "Sô ta, nghe không?",
        "Sô ta, lại đây",
        "Sô ta, giúp mình với",
        "Sô ta, bắt đầu nhé",
        "Sô ta, dừng lại",
        "Sô ta, tiếp tục đi",
        "Sô ta, mở bản đồ",
        "Sô ta, dẫn đường nhé",
        "Sô ta, quay lại",
        "Sô ta, đứng yên",
        "Sô ta, chào thầy đi",
        "Sô ta, tới đây nào",
        "Sô ta, cho mình hỏi",
        "Sô ta, nghe này",
        "Sô ta, trả lời mình",
        "Sô ta, nhìn bên này",
        "Sô ta, cảm ơn nhé",
        "Sô ta, cho tôi hỏi",
        "Bạn Sô ta ơi",
        "À, Sô ta đây rồi",
        "Sô ta, có nghe rõ không?",
        "Sô ta ơi, đi thôi",
    ),
    "hard_negative": (
        "Sổ tay",
        "Cái sổ tay ở đâu?",
        "Tôi để quên sổ tay",
        "Cho mình mượn cuốn sổ tay",
        "Quyển sổ tay màu đỏ",
        "Đừng quên mang sổ tay",
        "Cô ta",
        "Cô ta đang đứng đó",
        "Tôi vừa gặp cô ta",
        "Cô ta có tới không?",
        "Bạn bảo cô ta đi",
        "Cô ta đi qua cửa rồi",
        "Sô pha",
        "Ghế sô pha màu xanh",
        "Ngồi xuống ghế sô pha đi",
        "Đổi chiếc sô pha này nhé",
        "Soda",
        "Cho tôi một lon soda",
        "Bạn thích nước soda không?",
        "Số tám",
        "Đọc số tám nhé",
        "Viết số tám lên bảng",
        "Số tám ở góc phải",
        "Thôi ta đi",
        "Thôi ta về nào",
        "Chỗ ta đứng rất rộng",
        "Cái tô to ở đây",
        "Cho tôi cái tô to",
        "Còn sót à?",
        "Có gì bị sót à?",
    ),
    "negative": (
        "Xin chào mọi người",
        "Tôi đang ở trong phòng",
        "Cho tôi hỏi một chút",
        "Bạn có rảnh không?",
        "Cửa phòng đã mở chưa?",
        "Vui lòng bật đèn lên",
        "Chúng ta bắt đầu nhé",
        "Robot đang ở đâu?",
        "Tôi cần tìm phòng họp",
        "Thầy có ở trong phòng không?",
        "Hôm nay trời nóng quá",
        "Bạn có thể giúp tôi không?",
        "Cho tôi xin một cốc nước",
        "Mình muốn về phòng thí nghiệm",
        "Chờ tôi một chút nhé",
        "Bạn đang làm gì đấy?",
        "Xin phép đi qua",
        "Quay sang bên trái",
        "Dừng lại ở đây",
        "Đi thẳng tới cuối hành lang",
        "Hệ thống đã sẵn sàng",
        "Tôi sẽ quay lại sau",
        "Chúng mình đi thôi",
        "Tôi muốn kiểm tra micro",
        "Cảm ơn bạn nhiều",
        "Chúc một ngày tốt lành",
    ),
    "ambient": (
        "Không đọc lời: ghi tiếng quạt đang chạy trong phòng.",
        "Không đọc lời: ghi tiếng điều hòa, máy lọc không khí hoặc máy tính.",
        "Không đọc lời: ghi tiếng gõ bàn phím hoặc chuột.",
        "Không đọc lời: ghi tiếng bước chân ngoài hành lang.",
        "Không đọc lời: ghi tiếng cửa mở và đóng tự nhiên.",
        "Không đọc lời: ghi tiếng phòng học hoặc phòng họp lúc có người nói chuyện xa.",
        "Không đọc lời: ghi tiếng động cơ hoặc bánh xe robot đang chạy.",
        "Không đọc lời: ghi tiếng môi trường yên tĩnh, có âm nền tự nhiên.",
    ),
}

PROMPT_GUIDANCE = {
    "positive": "Phải đọc đúng tên gọi “Sô ta”; không thay bằng “soda”, “xô ta” hoặc tên khác. Câu dài nên chọn 5 giây.",
    "hard_negative": "Đọc đúng câu gây nhiễu, phát âm tự nhiên. Không vô tình nói đúng “Sô ta”.",
    "negative": "Đọc câu hội thoại tự nhiên; không chứa từ gọi robot “Sô ta”.",
    "ambient": "KHÔNG đọc dòng hướng dẫn. Chọn 30–60 giây, ghi âm thanh môi trường thực tế.",
}

class PromptCycler:
    """Keep separate prompt positions for each label. UI-only state, not registry."""

    def __init__(self):
        self._positions = {label: 0 for label in PROMPTS}

    def current(self, label):
        return PROMPTS[label][self._positions[label]]

    def position(self, label):
        return self._positions[label] + 1, len(PROMPTS[label])

    def advance(self, label):
        self._positions[label] = (self._positions[label] + 1) % len(PROMPTS[label])
        return self.current(label)
