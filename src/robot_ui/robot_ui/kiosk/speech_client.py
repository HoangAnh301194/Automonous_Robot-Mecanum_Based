"""Speech-to-Text (Groq Whisper) and Text-to-Speech (9Router Edge TTS) client."""
from __future__ import annotations

import json
import logging
import os
import re
import urllib.error
import urllib.request
from typing import Any

from .pronunciation import normalize_pronunciation

LOG = logging.getLogger(__name__)

# Default endpoints and models
DEFAULT_GROQ_STT_URL = "https://api.groq.com/openai/v1/audio/transcriptions"
DEFAULT_GROQ_MODEL = "whisper-large-v3-turbo"

DEFAULT_TTS_BASE_URL = os.getenv("KIOSK_TTS_BASE_URL", "http://127.0.0.1:20128/v1")
DEFAULT_TTS_MODEL_VI = "edge-tts/vi-VN-HoaiMyNeural"
DEFAULT_TTS_MODEL_EN = "edge-tts/en-US-JennyNeural"
DEFAULT_TTS_MODEL_JA = "edge-tts/ja-JP-NanamiNeural"
DEFAULT_TTS_MODEL_ZH = "edge-tts/zh-CN-XiaoyiNeural"
DEFAULT_TTS_MODEL_KO = "edge-tts/ko-KR-JiMinNeural"

VOICE_MAP = {
    "vi": DEFAULT_TTS_MODEL_VI,
    "en": DEFAULT_TTS_MODEL_EN,
    "ja": DEFAULT_TTS_MODEL_JA,
    "zh": DEFAULT_TTS_MODEL_ZH,
    "ko": DEFAULT_TTS_MODEL_KO,
}

# Common Whisper hallucinations when receiving low-energy or silent audio
HALLUCINATION_PATTERNS = [
    r"subscribe",
    r"ghiền mì gõ",
    r"đăng ký kênh",
    r"cảm ơn các bạn đã theo dõi",
    r"cảm ơn các bạn đã xem",
    r"chúc các bạn",
    r"hẹn gặp lại",
    r"thanks?\s+for\s+watching",
    r"thank\s+you\s+for\s+watching",
    r"like and subscribe",
    r"thanks?\s+for\s+listening",
    r"subtitles by",
]


def clean_markdown_for_tts(text: str) -> str:
    """Strip markdown syntax, raw URLs, source citations, and control symbols for smooth TTS."""
    if not text:
        return ""
    # Replace markdown links [text](url) -> text
    cleaned = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)
    # Remove raw URLs
    cleaned = re.sub(r"https?://\S+", "", cleaned)
    # Remove emails
    cleaned = re.sub(r"\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b", "", cleaned)
    # Strip source citations: e.g. "Nguồn: fee1_....txt", "Source: ...", "*Nguồn: ...*"
    cleaned = re.sub(r"(?i)\*?\s*(?:nguồn|source|tài liệu|tham khảo)\s*:\s*.*", "", cleaned)
    # Strip file names: e.g. "fee1_faculty_hoang_son_faq.txt", "abc.pdf"
    cleaned = re.sub(r"\b[\w\.-]+\.(?:txt|pdf|md|docx?)\b", "", cleaned)
    # Clean up empty label lines like "Email công tác:" or "Hồ sơ chính thức:"
    cleaned = re.sub(r"(?i)\b(?:email(?: công tác)?|hồ sơ(?: chính thức)?|website|link|địa chỉ web|liên hệ)\s*[:：]\s*(?=[.,;!?]|$|\s)", "", cleaned)
    # Remove markdown header, bold, italic, inline code characters
    cleaned = re.sub(r"[*_~`#>]", " ", cleaned)
    # Remove list bullet marks
    cleaned = re.sub(r"^\s*[-+*•]\s+", "", cleaned, flags=re.MULTILINE)
    # Strip parenthesized and bracketed annotations (e.g. (Souta), (phiên âm), （ソウタ）, 【chú thích】)
    without_annotations = re.sub(r"[\(\uff08\[\u3010][^\)\uff09\]\u3011]*[\)\uff09\]\u3011]", "", cleaned)
    if without_annotations.strip():
        cleaned = without_annotations
    # Clean up duplicate punctuation
    cleaned = re.sub(r"\s+([,.;!?])", r"\1", cleaned)
    cleaned = re.sub(r"([,.;!?])\s*\1+", r"\1", cleaned)
    # Normalize multiple whitespace and newlines
    cleaned = re.sub(r"\s+", " ", cleaned).strip()
    return cleaned


def detect_language(text: str, hint: str | None = None) -> str:
    """Detect language among vi, en, ja, zh, ko with priority and fallback."""
    if not text:
        return hint or "vi"

    # 1. Korean (Hangul syllables, Jamo)
    if re.search(r"[\uac00-\ud7af\u1100-\u11ff\u3130-\u318f]", text):
        return "ko"

    # 2. Japanese (Hiragana / Katakana)
    if re.search(r"[\u3040-\u309f\u30a0-\u30ff]", text):
        return "ja"

    # 3. Chinese (Hanzi without Kana)
    if re.search(r"[\u4e00-\u9fff]", text):
        return "zh"

    # 4. Vietnamese (diacritics)
    vi_regex = r"[àáạảãâầấậẩẫăằắặẳẵèéẹẻẽêềếệểễìíịỉĩòóọỏõôồốộổỗơờớợởỡùúụủũưừứựửữỳýỵỷỹđ]"
    if re.search(vi_regex, text, re.IGNORECASE):
        return "vi"

    # Check hint if text is purely Latin/English or numbers
    if hint in VOICE_MAP:
        return hint

    # Default to English if Latin characters dominate, or Vietnamese fallback
    if re.search(r"[A-Za-z]", text):
        return "en"

    return "vi"


def resolve_voice_and_text(text: str, language_hint: str | None = None) -> tuple[str, str]:
    """
    Resolve (voice_id, normalized_tts_text).
    Applies markdown cleanup, language detection, and technical pronunciation normalization.
    """
    cleaned = clean_markdown_for_tts(text)
    lang = detect_language(cleaned, hint=language_hint)
    voice_id = VOICE_MAP.get(lang, DEFAULT_TTS_MODEL_VI)

    # Normalize technical acronyms and titles according to language dictionary
    tts_text = normalize_pronunciation(cleaned, language=lang)

    # Truncate text if extremely long to prevent network timeout
    if len(tts_text) > 1500:
        tts_text = tts_text[:1500].rsplit(" ", 1)[0] + "..."

    return voice_id, tts_text


class SentenceSegmenter:
    """Incremental sentence splitter with safety checks for titles, decimals, and abbreviations."""

    _SENTENCE_END_RE = re.compile(r'([.!?]+(?:\s+|\n+)|[。！？])')
    _TITLE_PREFIX_RE = re.compile(
        r'(?:\b(?:TS|T\.S|ThS|Th\.S|PGS|P\.GS|GS|G\.S|Dr|Mr|Mrs|Ms|Prof|v\.v|etc|approx|e\.g|i\.e)\.)$',
        re.IGNORECASE,
    )
    _DECIMAL_RE = re.compile(r'\b\d+\.$')

    def __init__(self, min_sentence_chars: int = 5) -> None:
        self.min_chars = min_sentence_chars
        self._buffer = ""

    def push_delta(self, delta: str) -> list[str]:
        """Push a new text delta and return any newly completed sentences."""
        if not delta:
            return []
        self._buffer += delta
        return self._extract_sentences()

    def flush(self) -> list[str]:
        """Flush remaining text buffer as the final sentence."""
        remaining = self._buffer.strip()
        self._buffer = ""
        if remaining:
            return [remaining]
        return []

    def _extract_sentences(self) -> list[str]:
        completed: list[str] = []
        while True:
            dnl = self._buffer.find("\n\n")
            if dnl != -1 and dnl >= self.min_chars:
                candidate = self._buffer[:dnl].strip()
                self._buffer = self._buffer[dnl + 2:]
                if candidate:
                    completed.append(candidate)
                continue

            match = None
            for m in self._SENTENCE_END_RE.finditer(self._buffer):
                prefix = self._buffer[:m.end()].strip()
                if self._TITLE_PREFIX_RE.search(prefix) or self._DECIMAL_RE.search(prefix):
                    continue
                match = m
                break

            if not match:
                break

            split_pos = match.end()
            candidate = self._buffer[:split_pos].strip()
            self._buffer = self._buffer[split_pos:]

            if len(candidate) >= self.min_chars:
                completed.append(candidate)
            elif candidate:
                self._buffer = candidate + " " + self._buffer
                break

        return completed


def is_hallucination(text: str) -> bool:
    """Check if transcribed text is an empty or common Whisper hallucination."""
    normalized = text.strip().lower()
    if len(normalized) < 2:
        return True
    # Strip punctuation
    stripped = re.sub(r"[^\w\s]", "", normalized).strip()
    if len(stripped) < 2:
        return True
    for pattern in HALLUCINATION_PATTERNS:
        if re.search(pattern, stripped):
            LOG.warning("Hallucination detected and filtered: %s", text)
            return True
    return False


def get_groq_api_key() -> str:
    """Retrieve Groq API key from environment, with optional local fallback."""
    key = os.getenv("GROQ_API_KEY", "").strip()
    if key:
        return key
    # Fallback to sqlite database if present
    db_path = os.path.expanduser("~/.9router/db/data.sqlite")
    if os.path.isfile(db_path):
        try:
            import sqlite3
            con = sqlite3.connect(db_path)
            row = con.execute("SELECT data FROM providerConnections WHERE provider='groq';").fetchone()
            if row and row[0]:
                conn_data = json.loads(row[0])
                api_key = conn_data.get("apiKey", "")
                if api_key:
                    return api_key
        except Exception:
            pass
    return ""


def transcribe_audio_groq(
    wav_bytes: bytes,
    api_key: str | None = None,
    language: str | None = None,
    model: str = DEFAULT_GROQ_MODEL,
    timeout: float = 15.0,
) -> str:
    """
    Transcribe WAV audio using Groq Whisper API.
    Returns cleaned Unicode text or empty string if hallucination/error.
    """
    key = api_key or get_groq_api_key()
    if not key:
        raise RuntimeError("Thiếu cấu hình GROQ_API_KEY cho STT.")

    import requests

    headers = {
        "Authorization": f"Bearer {key}",
    }
    files = {
        "file": ("speech.wav", wav_bytes, "audio/wav"),
    }
    data: dict[str, Any] = {
        "model": model or DEFAULT_GROQ_MODEL,
        "response_format": "json",
    }
    # If language is explicitly provided and not auto, specify it
    lang_cfg = language or os.getenv("GROQ_STT_LANGUAGE", "auto").strip()
    if lang_cfg and lang_cfg != "auto":
        data["language"] = lang_cfg

    try:
        resp = requests.post(
            DEFAULT_GROQ_STT_URL,
            headers=headers,
            files=files,
            data=data,
            timeout=timeout,
        )
    except Exception as exc:
        LOG.exception("Groq STT request connection error: %s", exc)
        raise RuntimeError(f"Lỗi kết nối STT: {exc}") from exc

    if resp.status_code != 200:
        LOG.error("Groq STT returned HTTP %d: %s", resp.status_code, resp.text)
        if resp.status_code in (401, 403):
            raise RuntimeError("Khóa API Groq không hợp lệ hoặc hết hạn.")
        if resp.status_code == 429:
            raise RuntimeError("Hạn mức Groq API bị quá tải (Rate limit).")
        raise RuntimeError(f"Groq STT trả lỗi HTTP {resp.status_code}.")

    try:
        result = resp.json()
        transcript = result.get("text", "").strip()
    except Exception as exc:
        raise RuntimeError("Phản hồi STT không phải JSON hợp lệ.") from exc

    if is_hallucination(transcript):
        LOG.info("Transcript discarded as hallucination or empty: '%s'", transcript)
        return ""

    return transcript


def synthesize_speech_9router(
    text: str,
    voice_id: str | None = None,
    base_url: str = DEFAULT_TTS_BASE_URL,
    api_key: str | None = None,
    timeout: float = 15.0,
) -> bytes:
    """
    Synthesize speech using 9Router Edge-TTS endpoint.
    Returns raw audio bytes (MP3/WAV).
    """
    key = api_key or os.getenv("LLM_API_KEY", "")
    voice, processed_text = resolve_voice_and_text(text) if voice_id is None else (voice_id, text)

    if not processed_text.strip():
        return b""

    endpoint = base_url.rstrip("/") + "/audio/speech"
    payload = {
        "model": voice,
        "input": processed_text,
    }
    req = urllib.request.Request(
        endpoint,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {key}",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            audio_bytes = response.read()
            return audio_bytes
    except urllib.error.HTTPError as exc:
        LOG.error("TTS HTTP Error %d: %s", exc.code, exc.reason)
        raise RuntimeError(f"TTS trả lỗi HTTP {exc.code}.") from exc
    except (urllib.error.URLError, TimeoutError) as exc:
        LOG.error("TTS Connection Error: %s", exc)
        raise RuntimeError("Không thể kết nối dịch vụ TTS.") from exc
