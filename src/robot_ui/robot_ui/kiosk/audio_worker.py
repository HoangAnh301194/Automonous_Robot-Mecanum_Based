"""VoiceController: state machine and session coordinator for Kiosk Voice Chat."""
from __future__ import annotations

import logging
import os
import threading
import time
from enum import Enum
from typing import Any, Callable

from .audio_capture import AudioCapture
from .audio_player import AudioPlayer
from .speech_client import (
    resolve_voice_and_text,
    synthesize_speech_9router,
    transcribe_audio_groq,
)

LOG = logging.getLogger(__name__)


class VoiceState(Enum):
    READY = "ready"
    LISTENING = "listening"
    STT_PROCESSING = "stt_processing"
    LLM_PROCESSING = "llm_processing"
    TTS_PROCESSING = "tts_processing"
    SPEAKING = "speaking"
    UNAVAILABLE = "unavailable"
    ERROR = "error"
    CANCELLED = "cancelled"


class VoiceController:
    """Coordinates microphone recording, Groq STT, LLM response TTS, and mpv playback."""

    def __init__(
        self,
        on_state_changed: Callable[[VoiceState, str], None] | None = None,
        on_transcript_ready: Callable[[int, int, str], None] | None = None,
        on_tts_started: Callable[[int, int], None] | None = None,
        on_tts_finished: Callable[[int, int], None] | None = None,
        tts_base_url: str | None = None,
    ) -> None:
        self.on_state_changed = on_state_changed          # (state, status_message)
        self.on_transcript_ready = on_transcript_ready    # (session_id, turn_id, transcript)
        self.on_tts_started = on_tts_started              # (session_id, turn_id)
        self.on_tts_finished = on_tts_finished            # (session_id, turn_id)

        self.tts_base_url = tts_base_url or os.getenv("KIOSK_TTS_BASE_URL", "http://127.0.0.1:20128/v1")
        self.tts_enabled = os.getenv("KIOSK_TTS_ENABLED", "true").lower() in ("true", "1", "yes")

        self._lock = threading.Lock()
        self._session_id = 0
        self._turn_id = 0
        self._state = VoiceState.READY
        self._speaker_enabled = True

        # Components
        self.capture = AudioCapture(
            on_audio_ready=self._on_audio_captured,
            on_no_speech=self._on_no_speech,
            on_error=self._on_capture_error,
        )
        self.player = AudioPlayer(
            on_playback_started=self._on_playback_started,
            on_playback_finished=self._on_playback_finished,
            on_playback_error=self._on_playback_error,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def session_id(self) -> int:
        with self._lock:
            return self._session_id

    @property
    def turn_id(self) -> int:
        with self._lock:
            return self._turn_id

    @property
    def state(self) -> VoiceState:
        with self._lock:
            return self._state

    @property
    def is_speaker_enabled(self) -> bool:
        with self._lock:
            return self._speaker_enabled

    @property
    def is_active(self) -> bool:
        """True if the controller is in any active turn state (cannot idle-reset)."""
        with self._lock:
            return self._state in (
                VoiceState.LISTENING,
                VoiceState.STT_PROCESSING,
                VoiceState.LLM_PROCESSING,
                VoiceState.TTS_PROCESSING,
                VoiceState.SPEAKING,
            )

    # ------------------------------------------------------------------
    # Public Control API
    # ------------------------------------------------------------------

    def set_speaker_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._speaker_enabled = enabled
        self.player.set_muted(not enabled)
        if not enabled and self._state == VoiceState.SPEAKING:
            self.stop_speaking()

    def set_volume(self, volume: int) -> None:
        self.player.set_volume(volume)

    def get_volume(self) -> int:
        return self.player.volume

    def toggle_listening(self) -> None:
        """Toggle recording: if listening, stops and submits; otherwise starts recording."""
        with self._lock:
            if self._state == VoiceState.LISTENING:
                self.capture.stop()
                return

        # Stop ongoing speech before listening (half-duplex)
        if self._state == VoiceState.SPEAKING:
            self.stop_speaking()

        self.start_listening()

    def start_listening(self) -> bool:
        with self._lock:
            self._turn_id += 1
            session_id = self._session_id
            turn_id = self._turn_id

        self._set_state(VoiceState.LISTENING, "Đang nghe... Chạm lần nữa để gửi")
        success = self.capture.start()
        if not success:
            self._set_state(VoiceState.UNAVAILABLE, "Không tìm thấy microphone")
            # Automatically restore to ready after 2.5s
            threading.Timer(2.5, lambda: self._restore_ready_if_state(VoiceState.UNAVAILABLE)).start()
            return False
        return True

    def stop_listening(self) -> None:
        """Manually stop recording and begin STT processing."""
        self.capture.stop()

    def stop_speaking(self) -> None:
        """Stop current speech synthesis and mpv playback immediately."""
        self.player.stop()
        with self._lock:
            if self._state in (VoiceState.TTS_PROCESSING, VoiceState.SPEAKING):
                self._state = VoiceState.READY
        self._emit_state("Sẵn sàng trò chuyện")

    def reset_session(self) -> None:
        """Invalidate all in-flight STT/TTS callbacks and abort playback/capture."""
        with self._lock:
            self._session_id += 1
            self._turn_id += 1
            self._state = VoiceState.READY
        self.capture.cancel()
        self.player.stop()
        self._emit_state("Sẵn sàng trò chuyện")

    def notify_llm_processing(self) -> None:
        """Called when LLM begins thinking for a user query."""
        self._set_state(VoiceState.LLM_PROCESSING, "Đang xử lý câu hỏi...")

    def handle_llm_response(self, session_id: int, text: str) -> None:
        """Called when LLM returns an assistant response."""
        with self._lock:
            if session_id != self._session_id:
                LOG.info("Discarding stale LLM response from prior session.")
                return
            if not self._speaker_enabled:
                self._state = VoiceState.READY
                self._emit_state("Sẵn sàng trò chuyện")
                return
            turn_id = self._turn_id

        # Trigger TTS in background thread
        threading.Thread(
            target=self._synthesize_and_play,
            args=(session_id, turn_id, text),
            name=f"tts-worker-{turn_id}",
            daemon=True,
        ).start()

    def shutdown(self) -> None:
        """Gracefully release all audio hardware and subprocesses."""
        self.capture.cancel()
        self.player.shutdown()

    # ------------------------------------------------------------------
    # Internal Pipeline
    # ------------------------------------------------------------------

    def _set_state(self, state: VoiceState, msg: str) -> None:
        with self._lock:
            self._state = state
        self._emit_state(msg)

    def _emit_state(self, msg: str) -> None:
        if self.on_state_changed:
            self.on_state_changed(self._state, msg)

    def _restore_ready_if_state(self, expected_state: VoiceState) -> None:
        with self._lock:
            if self._state == expected_state:
                self._state = VoiceState.READY
                self._emit_state("Sẵn sàng trò chuyện")

    def _on_audio_captured(self, wav_bytes: bytes, duration: float) -> None:
        with self._lock:
            session_id = self._session_id
            turn_id = self._turn_id

        self._set_state(VoiceState.STT_PROCESSING, "Đang nhận dạng giọng nói...")

        threading.Thread(
            target=self._transcribe_worker,
            args=(session_id, turn_id, wav_bytes),
            name=f"stt-worker-{turn_id}",
            daemon=True,
        ).start()

    def _on_no_speech(self) -> None:
        LOG.info("No speech detected.")
        self._set_state(VoiceState.READY, "Không phát hiện tiếng nói")
        threading.Timer(2.0, lambda: self._restore_ready_if_state(VoiceState.READY)).start()

    def _on_capture_error(self, err_msg: str) -> None:
        LOG.error("Microphone capture error: %s", err_msg)
        self._set_state(VoiceState.ERROR, err_msg)
        threading.Timer(2.5, lambda: self._restore_ready_if_state(VoiceState.ERROR)).start()

    def _transcribe_worker(self, session_id: int, turn_id: int, wav_bytes: bytes) -> None:
        try:
            transcript = transcribe_audio_groq(wav_bytes)
        except Exception as exc:
            LOG.exception("STT request failed: %s", exc)
            with self._lock:
                if session_id != self._session_id or turn_id != self._turn_id:
                    return
            self._set_state(VoiceState.ERROR, "Không thể kết nối STT")
            threading.Timer(2.5, lambda: self._restore_ready_if_state(VoiceState.ERROR)).start()
            return

        with self._lock:
            if session_id != self._session_id or turn_id != self._turn_id:
                LOG.info("Discarding stale STT transcript.")
                return

        if not transcript:
            self._set_state(VoiceState.READY, "Không nghe rõ, vui lòng nói lại")
            threading.Timer(2.5, lambda: self._restore_ready_if_state(VoiceState.READY)).start()
            return

        LOG.info("STT transcript ready: '%s'", transcript)
        # Notify UI to submit transcript to conversation
        if self.on_transcript_ready:
            self.on_transcript_ready(session_id, turn_id, transcript)

    def _synthesize_and_play(self, session_id: int, turn_id: int, text: str) -> None:
        self._set_state(VoiceState.TTS_PROCESSING, "Đang tạo giọng nói...")
        if self.on_tts_started:
            self.on_tts_started(session_id, turn_id)

        try:
            voice_id, tts_text = resolve_voice_and_text(text)
            audio_data = synthesize_speech_9router(
                tts_text,
                voice_id=voice_id,
                base_url=self.tts_base_url,
            )
        except Exception as exc:
            LOG.exception("TTS synthesis failed: %s", exc)
            with self._lock:
                if session_id != self._session_id or turn_id != self._turn_id:
                    return
            self._set_state(VoiceState.READY, "Không thể tạo giọng nói")
            if self.on_tts_finished:
                self.on_tts_finished(session_id, turn_id)
            return

        with self._lock:
            if session_id != self._session_id or turn_id != self._turn_id:
                LOG.info("Discarding stale TTS audio.")
                return
            if not self._speaker_enabled:
                self._state = VoiceState.READY
                self._emit_state("Sẵn sàng trò chuyện")
                if self.on_tts_finished:
                    self.on_tts_finished(session_id, turn_id)
                return

        self._set_state(VoiceState.SPEAKING, "Souta đang nói...")
        played = self.player.play(audio_data, request_id=turn_id)
        if not played:
            self._set_state(VoiceState.READY, "Sẵn sàng trò chuyện")
            if self.on_tts_finished:
                self.on_tts_finished(session_id, turn_id)

    def _on_playback_started(self, request_id: int) -> None:
        LOG.info("Playback started (turn_id=%d)", request_id)

    def _on_playback_finished(self, request_id: int) -> None:
        LOG.info("Playback finished (turn_id=%d)", request_id)
        with self._lock:
            if self._state == VoiceState.SPEAKING:
                self._state = VoiceState.READY
        self._emit_state("Sẵn sàng trò chuyện")
        if self.on_tts_finished:
            self.on_tts_finished(self._session_id, request_id)

    def _on_playback_error(self, request_id: int, err_msg: str) -> None:
        LOG.error("Playback error (turn_id=%d): %s", request_id, err_msg)
        with self._lock:
            self._state = VoiceState.READY
        self._emit_state("Lỗi phát âm thanh")
        if self.on_tts_finished:
            self.on_tts_finished(self._session_id, request_id)
