"""Unit and GUI integration tests for Souta Robot Voice Chat."""
from __future__ import annotations

import os
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.audio_capture import AudioCapture, calculate_rms, pcm_to_wav
from robot_ui.kiosk.audio_player import AudioPlayer
from robot_ui.kiosk.audio_worker import VoiceController, VoiceState
from robot_ui.kiosk.speech_client import (
    clean_markdown_for_tts,
    detect_language,
    is_hallucination,
    resolve_voice_and_text,
    synthesize_speech_9router,
    transcribe_audio_groq,
)
from robot_ui.kiosk.widgets import ChatPage, MicState


@pytest.fixture(scope="session")
def app():
    app = QApplication.instance()
    if app is None:
        app = QApplication(["kiosk-voice-test"])
    return app


# ======================================================================
# 1. Audio Processing & Speech Client Tests
# ======================================================================

def test_pcm_to_wav_and_rms():
    # 0.1s of silence (1600 samples, 3200 bytes of zeros)
    pcm_silence = b"\x00" * 3200
    rms_zero = calculate_rms(pcm_silence)
    assert rms_zero == 0.0

    wav = pcm_to_wav(pcm_silence, sample_rate=16000, channels=1)
    assert len(wav) == 3200 + 44
    assert wav[:4] == b"RIFF"
    assert wav[8:12] == b"WAVE"


def test_hallucination_filter():
    assert is_hallucination("")
    assert is_hallucination("   ")
    assert is_hallucination("a")
    assert is_hallucination("...")
    assert is_hallucination("Hãy subscribe cho kênh Ghiền Mì Gõ")
    assert is_hallucination("Thanks for watching!")
    assert is_hallucination("Cảm ơn các bạn đã theo dõi.")
    # Valid phrases must not be flagged
    assert not is_hallucination("Xin chào Souta")
    assert not is_hallucination("Khoa Kỹ thuật Điện tử 1 ở đâu?")
    assert not is_hallucination("Hello, who are you?")
    assert not is_hallucination("こんにちは、元気ですか？")


def test_markdown_cleaning_for_tts():
    raw = "Xin chào **Souta**! Bạn có thể xem [tại đây](https://ptit.edu.vn). *Cảm ơn* (chú thích). 【Ghi chú】"
    cleaned = clean_markdown_for_tts(raw)
    assert "https://" not in cleaned
    assert "[" not in cleaned
    assert "**" not in cleaned
    assert "(chú thích)" not in cleaned
    assert "【Ghi chú】" not in cleaned
    assert "Xin chào Souta" in cleaned


def test_annotation_stripping_for_cjk_tts():
    raw = "こんにちは (Xin chào), わたしはソウタです（Souta Robot）。【phát âm】"
    cleaned = clean_markdown_for_tts(raw)
    assert "(Xin chào)" not in cleaned
    assert "（Souta Robot）" not in cleaned
    assert "【phát âm】" not in cleaned
    assert "こんにちは" in cleaned


def test_language_detection_and_voice_routing_5_languages():
    # 1. Vietnamese (detect diacritics and phonetics fix Souta -> Sota)
    voice_vi, text_vi = resolve_voice_and_text("Chào bạn, tôi là Souta Robot của PTIT.")
    assert voice_vi == "edge-tts/vi-VN-HoaiMyNeural"
    assert "Sota" in text_vi
    assert "Souta" not in text_vi

    # 2. English
    voice_en, text_en = resolve_voice_and_text("Hello, how are you today?")
    assert voice_en == "edge-tts/en-US-JennyNeural"
    assert "Hello" in text_en

    # 3. Japanese (Hiragana/Katakana)
    voice_ja, text_ja = resolve_voice_and_text("こんにちは、ソウタロボットです。")
    assert voice_ja == "edge-tts/ja-JP-NanamiNeural"

    # 4. Chinese (Hanzi without Kana)
    voice_zh, text_zh = resolve_voice_and_text("你好，我是Souta机器人。很高兴认识你！")
    assert voice_zh == "edge-tts/zh-CN-XiaoyiNeural"

    # 5. Korean (Hangul)
    voice_ko, text_ko = resolve_voice_and_text("안녕하세요, 저는 소우타 로봇입니다.")
    assert voice_ko == "edge-tts/ko-KR-JiMinNeural"


def test_stt_missing_api_key_raises_runtime_error(monkeypatch):
    monkeypatch.setenv("GROQ_API_KEY", "")
    with patch("os.path.isfile", return_value=False):
        with pytest.raises(RuntimeError, match="Thiếu cấu hình GROQ_API_KEY"):
            transcribe_audio_groq(b"RIFF...", api_key="")


# ======================================================================
# 2. Audio Player Tests (mpv PID Management & Cancellation)
# ======================================================================

def test_audio_player_play_stop_mute():
    import time
    player = AudioPlayer()
    assert not player.is_playing()

    # Muted player will not start playback
    player.set_muted(True)
    assert player.is_muted
    assert not player.play(b"fake_mp3_data", request_id=1)

    player.set_muted(False)
    assert not player.is_muted

    # Mock subprocess.Popen for playback
    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_proc.wait.side_effect = lambda *args, **kwargs: time.sleep(0.5)
    with patch("robot_ui.kiosk.audio_player.subprocess.Popen", return_value=mock_proc):
        played = player.play(b"fake_mp3_data", request_id=42)
        assert played
        time.sleep(0.02)
        # Stopping terminates specific proc
        player.stop()
        mock_proc.terminate.assert_called()


def test_audio_player_no_overlapping_playback():
    import time
    player = AudioPlayer()
    mock_proc1 = MagicMock()
    mock_proc1.poll.return_value = None
    mock_proc1.wait.side_effect = lambda *args, **kwargs: time.sleep(0.5)

    mock_proc2 = MagicMock()
    mock_proc2.poll.return_value = None
    mock_proc2.wait.side_effect = lambda *args, **kwargs: time.sleep(0.5)

    with patch("robot_ui.kiosk.audio_player.subprocess.Popen", side_effect=[mock_proc1, mock_proc2]):
        player.play(b"audio1", request_id=1)
        time.sleep(0.02)
        # Playing second audio terminates first
        player.play(b"audio2", request_id=2)
        mock_proc1.terminate.assert_called()
        player.stop()
        mock_proc2.terminate.assert_called()


# ======================================================================
# 3. VoiceController State Machine & Pipeline Tests
# ======================================================================

def test_voice_controller_state_flow():
    states = []
    controller = VoiceController(
        on_state_changed=lambda st, msg: states.append((st, msg))
    )
    assert controller.state == VoiceState.READY

    with patch.object(controller.capture, "start", return_value=True):
        started = controller.start_listening()
        assert started
        assert controller.state == VoiceState.LISTENING

        # Second toggle stops listening
        with patch.object(controller.capture, "stop") as mock_stop:
            controller.toggle_listening()
            mock_stop.assert_called()


def test_voice_controller_microphone_unavailable():
    controller = VoiceController()
    with patch.object(controller.capture, "start", return_value=False):
        started = controller.start_listening()
        assert not started
        assert controller.state == VoiceState.UNAVAILABLE


def test_voice_controller_no_speech_returns_to_ready():
    controller = VoiceController()
    controller._on_no_speech()
    assert controller.state == VoiceState.READY


def test_voice_controller_stale_result_discarded():
    transcripts = []
    controller = VoiceController(
        on_transcript_ready=lambda s, t, txt: transcripts.append((s, t, txt))
    )
    # Session 0, turn 1
    controller._session_id = 0
    controller._turn_id = 1

    # Simulate reset before STT worker returns
    controller.reset_session()
    assert controller.session_id == 1

    # Now a late worker callback with old session 0 arrives
    with patch("robot_ui.kiosk.audio_worker.transcribe_audio_groq", return_value="Câu hỏi cũ"):
        controller._transcribe_worker(session_id=0, turn_id=1, wav_bytes=b"wav")

    assert len(transcripts) == 0  # Stale result must be discarded!


def test_voice_controller_speaker_off_skips_playback():
    controller = VoiceController()
    controller.set_speaker_enabled(False)

    with patch.object(controller.player, "play") as mock_play:
        controller.handle_llm_response(session_id=controller.session_id, text="Xin chào")
        mock_play.assert_not_called()


def test_voice_controller_stop_speaking():
    controller = VoiceController()
    with patch.object(controller.player, "stop") as mock_stop:
        controller._state = VoiceState.SPEAKING
        controller.stop_speaking()
        mock_stop.assert_called_once()
        assert controller.state == VoiceState.READY


# ======================================================================
# 4. GUI & ChatPage Integration Tests (Offscreen Qt)
# ======================================================================

def test_chat_page_icons_and_states(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    # Initial
    assert page._mic_state == MicState.UNAVAILABLE

    # READY
    page.set_mic_state(MicState.READY)
    assert page.mic_btn.isEnabled()
    assert page.mic_btn.property("listening") == "false"

    # LISTENING (pulse timer running)
    page.set_mic_state(MicState.LISTENING)
    assert page.mic_btn.isEnabled()
    assert page.mic_btn.property("listening") == "true"
    assert page._pulse_timer.isActive()

    # STT_PROCESSING (pulse stopped)
    page.set_mic_state(MicState.STT_PROCESSING)
    assert not page.mic_btn.isEnabled()
    assert not page._pulse_timer.isActive()

    # LLM_PROCESSING (shows stop button)
    page.set_mic_state(MicState.LLM_PROCESSING)
    assert not page.mic_btn.isEnabled()
    assert not page._stop_speaking_btn.isHidden()
    assert page._stop_speaking_btn.text() == "Dừng"

    # TTS_PROCESSING (shows stop speaking button)
    page.set_mic_state(MicState.TTS_PROCESSING)
    assert not page.mic_btn.isEnabled()
    assert not page._stop_speaking_btn.isHidden()

    # SPEAKING (shows stop speaking button)
    page.set_mic_state(MicState.SPEAKING)
    assert not page._stop_speaking_btn.isHidden()

    # Back to READY (hides stop speaking button)
    page.set_speaker_speaking(False)
    assert page._stop_speaking_btn.isHidden()


def test_submit_voice_transcript_emits_once(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    submitted = []
    page.chat_submitted.connect(lambda txt: submitted.append(txt))

    page.submit_voice_transcript("Chào robot Souta")
    assert len(submitted) == 1
    assert submitted[0] == "Chào robot Souta"

    # Empty voice transcript does not submit
    page.submit_voice_transcript("   ")
    assert len(submitted) == 1


def test_speaker_toggle_behavior(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    toggled = []
    page.speaker_toggled.connect(lambda en: toggled.append(en))

    assert page._speaker_enabled is True
    # Click speaker button toggles OFF
    page._speaker_btn.click()
    assert page._speaker_enabled is False
    assert toggled[-1] is False

    # Click again toggles ON
    page._speaker_btn.click()
    assert page._speaker_enabled is True
    assert toggled[-1] is True
