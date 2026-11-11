"""Unit tests for technical keyword pronunciation dictionary."""
from __future__ import annotations

import pytest
from robot_ui.kiosk.pronunciation import normalize_pronunciation
from robot_ui.kiosk.speech_client import resolve_voice_and_text


def test_iot_pronunciation_vietnamese():
    # IoT must be pronounced 'ai âu ti', never 'i o tê'
    text1 = "Souta có thể kết nối với các thiết bị IoT trong phòng lab."
    norm1 = normalize_pronunciation(text1, language="vi")
    assert "ai âu ti" in norm1
    assert "IoT" not in norm1

    # Case variations: IOT, iot
    assert "ai âu ti" in normalize_pronunciation("Hệ thống IOT thông minh", language="vi")
    assert "ai âu ti" in normalize_pronunciation("Ứng dụng iot trong tự động hóa", language="vi")


def test_ai_pronunciation_case_sensitive():
    # Uppercase AI -> ây ai
    text = "Nghiên cứu về AI và robotics."
    norm = normalize_pronunciation(text, language="vi")
    assert "ây ai" in norm

    # Lowercase 'ai' in Vietnamese (e.g., 'ai đó', 'không ai') must NOT be replaced!
    vi_pronoun = "Có ai ở trong phòng thí nghiệm không?"
    norm_vi = normalize_pronunciation(vi_pronoun, language="vi")
    assert "Có ai ở trong phòng" in norm_vi
    assert "ây ai" not in norm_vi


def test_technical_acronyms_pronunciation():
    text = "Robot giao tiếp qua API, trang bị CPU và GPU mạnh mẽ, kết nối cổng USB."
    norm = normalize_pronunciation(text, language="vi")
    assert "ây pi ai" in norm
    assert "xi pi iu" in norm
    assert "gi pi iu" in norm
    assert "iu ét bi" in norm


def test_word_boundary_safety():
    # Terms containing 'iot' or 'ai' as substrings must NOT be corrupted
    text = "Công nghệ BIOtech và ngành robotics phát triển mạnh."
    norm = normalize_pronunciation(text, language="vi")
    assert "BIOtech" in norm or "biotech" in norm.lower()
    assert "robotics" in norm
    assert "ai âu ti" not in norm


def test_academic_titles_and_institutions():
    text = "PGS.TS Hoàng Sơn là giảng viên Khoa FEE1 tại PTIT."
    norm = normalize_pronunciation(text, language="vi")
    assert "Phó giáo sư Tiến sĩ" in norm
    assert "Khoa Điện tử 1" in norm
    assert "Pi-ti-ai-ti" in norm


def test_english_not_affected_by_vietnamese_rules():
    # English sentences must preserve their original English words
    en_text = "I can introduce IoT and AI technology to visitors."
    norm_en = normalize_pronunciation(en_text, language="en")
    assert norm_en == en_text
    assert "ai âu ti" not in norm_en


def test_resolve_voice_and_text_integration():
    vi_text = "Souta là robot hỗ trợ IoT và AI tại PTIT."
    voice_id, tts_text = resolve_voice_and_text(vi_text)
    assert "HoaiMyNeural" in voice_id
    assert "Sota" in tts_text
    assert "ai âu ti" in tts_text
    assert "ây ai" in tts_text
    assert "Pi-ti-ai-ti" in tts_text
    # Original string is untouched
    assert "IoT" in vi_text
