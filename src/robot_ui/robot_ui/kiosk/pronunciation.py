"""Technical keyword pronunciation dictionary for TTS speech normalization."""
from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any

LOG = logging.getLogger(__name__)

CONFIG_PATH_CANDIDATES = [
    Path(__file__).resolve().parent.parent.parent.parent / "config" / "tts_pronunciation.json",
    Path(__file__).parent.parent / "config" / "tts_pronunciation.json",
    Path("/opt/ros/humble/share/robot_ui/config/tts_pronunciation.json"),
]

DEFAULT_VI_REPLACEMENTS: list[dict[str, Any]] = [
    {"pattern": r"\b(?:PGS|P\.GS)\.?\s*(?:TS|T\.S)\.?", "replacement": "Phó giáo sư Tiến sĩ", "case_sensitive": False},
    {"pattern": r"\b(?:GS|G\.S)\.?\s*(?:TS|T\.S)\.?", "replacement": "Giáo sư Tiến sĩ", "case_sensitive": False},
    {"pattern": r"\b(?:PGS|P\.GS)\.?", "replacement": "Phó giáo sư", "case_sensitive": False},
    {"pattern": r"\b(?:GS|G\.S)\.?", "replacement": "Giáo sư", "case_sensitive": False},
    {"pattern": r"\b(?:TS|T\.S)\.?", "replacement": "Tiến sĩ", "case_sensitive": True},
    {"pattern": r"\b(?:ThS|Th\.S)\.?", "replacement": "Thạc sĩ", "case_sensitive": True},
    {"pattern": r"\bPTIT\b", "replacement": "Pi-ti-ai-ti", "case_sensitive": False},
    {"pattern": r"\bFEE[-_]?1\b", "replacement": "Khoa Điện tử 1", "case_sensitive": False},
    {"pattern": r"\b[Ss]outa\b", "replacement": "Sota", "case_sensitive": False},
    {"pattern": r"\bIoT\b", "replacement": "ai âu ti", "case_sensitive": False},
    {"pattern": r"\bAI\b", "replacement": "ây ai", "case_sensitive": True},
    {"pattern": r"\bAPI\b", "replacement": "ây pi ai", "case_sensitive": False},
    {"pattern": r"\bCPU\b", "replacement": "xi pi iu", "case_sensitive": False},
    {"pattern": r"\bGPU\b", "replacement": "gi pi iu", "case_sensitive": False},
    {"pattern": r"\bUSB\b", "replacement": "iu ét bi", "case_sensitive": False},
]

_COMPILED_RULES: dict[str, list[tuple[re.Pattern, str]]] = {}


def load_pronunciation_rules() -> dict[str, list[tuple[re.Pattern, str]]]:
    """Load and compile pronunciation rules from JSON config or fallback defaults."""
    global _COMPILED_RULES
    if _COMPILED_RULES:
        return _COMPILED_RULES

    raw_data: dict[str, Any] = {}
    for p in CONFIG_PATH_CANDIDATES:
        try:
            if p.is_file():
                raw_data = json.loads(p.read_text(encoding="utf-8"))
                LOG.info("Loaded TTS pronunciation dictionary from %s", p)
                break
        except Exception as exc:
            LOG.warning("Failed to parse pronunciation config %s: %s", p, exc)

    compiled: dict[str, list[tuple[re.Pattern, str]]] = {}

    # Vietnamese rules
    vi_list = raw_data.get("vi", {}).get("replacements") or DEFAULT_VI_REPLACEMENTS
    compiled_vi: list[tuple[re.Pattern, str]] = []
    for item in vi_list:
        pat = item.get("pattern", "")
        repl = item.get("replacement", "")
        case_sensitive = item.get("case_sensitive", False)
        flags = 0 if case_sensitive else re.IGNORECASE
        try:
            compiled_vi.append((re.compile(pat, flags), repl))
        except re.error as exc:
            LOG.error("Invalid regex in pronunciation config: %s (%s)", pat, exc)

    compiled["vi"] = compiled_vi
    _COMPILED_RULES = compiled
    return _COMPILED_RULES


def normalize_pronunciation(text: str, language: str = "vi") -> str:
    """
    Normalize technical acronyms and names to spoken phonetics.
    Applies ONLY to text copy sent to TTS. UI text and history remain untouched.
    """
    if not text or language != "vi":
        return text

    rules = load_pronunciation_rules().get(language, [])
    result = text
    for regex, replacement in rules:
        result = regex.sub(replacement, result)

    result = re.sub(r"\s+([,.;!?])", r"\1", result)
    result = re.sub(r"\s+", " ", result).strip()
    return result
