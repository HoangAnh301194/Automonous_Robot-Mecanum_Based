"""LLM client for robot chatbot via 9router (OpenAI-compatible API).

Chạy trong QThread riêng để không block UI.
Hỗ trợ:
  - Gọi 9router tại http://localhost:20128/v1/chat/completions
  - RAG từ folder rag_docs/ (đọc tất cả file .txt)
  - Inject trạng thái robot vào system prompt
  - Trả về toàn bộ response (không streaming)
"""
from __future__ import annotations

import json
import os
import threading
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# Cấu hình mặc định – có thể override bằng biến môi trường
# ---------------------------------------------------------------------------
DEFAULT_BASE_URL = os.environ.get("LLM_BASE_URL", "http://localhost:20128/v1")
DEFAULT_API_KEY = os.environ.get(
    "LLM_API_KEY", "sk-67a7867f2cd97777-xaimwy-96411728"
)
DEFAULT_MODEL = os.environ.get("LLM_MODEL", "hehehe")  # combo model trong 9router dashboard
REQUEST_TIMEOUT = 30  # giây

RAG_DOCS_DIR = Path(__file__).parent / "rag_docs"

SYSTEM_PROMPT_TEMPLATE = """\
Bạn là Souta – trợ lý robot thông minh được phát triển tại PTIT.
Trả lời ngắn gọn, thân thiện, bằng tiếng Việt (trừ khi người dùng dùng ngôn ngữ khác).
Không bịa đặt thông tin ngoài tài liệu và dữ liệu được cung cấp.

--- TÀI LIỆU THAM KHẢO ---
{rag_context}

--- TRẠNG THÁI ROBOT HIỆN TẠI ---
{robot_context}
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_rag_docs(docs_dir: Path) -> str:
    """Đọc tất cả file .txt trong thư mục rag_docs và ghép thành 1 chuỗi."""
    if not docs_dir.exists():
        return "(Không có tài liệu tham khảo)"
    parts: list[str] = []
    for txt_file in sorted(docs_dir.glob("*.txt")):
        try:
            content = txt_file.read_text(encoding="utf-8").strip()
            if content:
                parts.append(f"[{txt_file.name}]\n{content}")
        except OSError:
            pass
    return "\n\n".join(parts) if parts else "(Không có tài liệu tham khảo)"


def _format_robot_context(robot_status: dict[str, Any]) -> str:
    """Chuyển dict robot_status thành chuỗi dễ đọc cho LLM."""
    if not robot_status:
        return "Chưa có dữ liệu trạng thái robot."
    lines = []
    mapping = {
        "robot_state": "Trạng thái",
        "current_location": "Vị trí hiện tại",
        "battery_percent": "Pin (%)",
        "battery_low": "Pin thấp",
        "lidar_ok": "LiDAR OK",
        "localization_ok": "Định vị OK",
        "odom_ok": "Odometry OK",
        "water_liter": "Nước còn (lít)",
        "water_low": "Nước thấp",
    }
    for key, label in mapping.items():
        if key in robot_status:
            val = robot_status[key]
            if isinstance(val, float):
                val = f"{val:.1f}"
            lines.append(f"{label}: {val}")
    return "\n".join(lines) if lines else "Chưa có dữ liệu."


# ---------------------------------------------------------------------------
# Core API call (blocking, runs in worker thread)
# ---------------------------------------------------------------------------

def call_llm(
    user_message: str,
    robot_status: dict[str, Any] | None = None,
    base_url: str = DEFAULT_BASE_URL,
    api_key: str = DEFAULT_API_KEY,
    model: str = DEFAULT_MODEL,
) -> str:
    """Gọi 9router API và trả về response text.

    Raises:
        RuntimeError: nếu API trả về lỗi hoặc network error.
    """
    rag_context = _load_rag_docs(RAG_DOCS_DIR)
    robot_context = _format_robot_context(robot_status or {})

    system_prompt = SYSTEM_PROMPT_TEMPLATE.format(
        rag_context=rag_context,
        robot_context=robot_context,
    )

    payload: dict[str, Any] = {
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message},
        ],
        "stream": False,
    }
    if model:
        payload["model"] = model

    data = json.dumps(payload).encode("utf-8")
    endpoint = base_url.rstrip("/") + "/chat/completions"

    req = urllib.request.Request(
        endpoint,
        data=data,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT) as resp:
            body = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code}: {raw}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Không thể kết nối đến LLM server: {exc.reason}") from exc
    except TimeoutError as exc:
        raise RuntimeError("Hết thời gian chờ phản hồi từ LLM server.") from exc

    try:
        return body["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(f"Phản hồi không hợp lệ từ API: {body}") from exc


# ---------------------------------------------------------------------------
# Non-blocking wrapper dùng PyQt signal
# ---------------------------------------------------------------------------

class LLMWorker:
    """Chạy call_llm trong thread riêng, gọi callback khi xong."""

    def __init__(
        self,
        on_response,   # callable(str)  – gọi khi thành công
        on_error,      # callable(str)  – gọi khi lỗi
        base_url: str = DEFAULT_BASE_URL,
        api_key: str = DEFAULT_API_KEY,
        model: str = DEFAULT_MODEL,
    ) -> None:
        self._on_response = on_response
        self._on_error = on_error
        self._base_url = base_url
        self._api_key = api_key
        self._model = model
        self._lock = threading.Lock()
        self._busy = False

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._busy

    def send(self, message: str, robot_status: dict[str, Any] | None = None) -> bool:
        """Gửi message. Trả về False nếu đang xử lý request khác."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True

        t = threading.Thread(
            target=self._run,
            args=(message, robot_status),
            daemon=True,
            name="llm-worker",
        )
        t.start()
        return True

    def _run(self, message: str, robot_status: dict[str, Any] | None) -> None:
        try:
            response = call_llm(
                message,
                robot_status=robot_status,
                base_url=self._base_url,
                api_key=self._api_key,
                model=self._model,
            )
            self._on_response(response)
        except RuntimeError as exc:
            self._on_error(str(exc))
        finally:
            with self._lock:
                self._busy = False
