"""VoiceController: state machine and session coordinator for Kiosk Voice Chat."""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from enum import Enum
from typing import Any, Callable

from .audio_capture import AudioCapture
from .audio_player import AudioPlayer
from .llm_client import TurnContext
from .speech_client import (
    SentenceSegmenter,
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
    """Coordinates microphone recording, Groq STT, incremental sentence TTS, and mpv playback."""

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
        self._generation_id = 0
        self._tts_generation_id = 0
        self._state = VoiceState.READY
        self._speaker_enabled = True

        # Streaming Sentence-by-Sentence TTS Queues
        self._segmenter = SentenceSegmenter(min_sentence_chars=6)
        self._segment_index = 0
        self._tts_queue: queue.Queue[tuple[TurnContext, int, str, bool]] = queue.Queue(maxsize=50)
        self._playback_queue: queue.Queue[tuple[TurnContext, int, bytes, bool]] = queue.Queue(maxsize=50)
        self._playback_done_event = threading.Event()
        self._shutdown_event = threading.Event()

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

        # Start background workers for sentence TTS synthesis and sequential playback
        self._tts_thread = threading.Thread(
            target=self._tts_worker_loop,
            name="voice-tts-worker",
            daemon=True,
        )
        self._tts_thread.start()

        self._playback_thread = threading.Thread(
            target=self._playback_worker_loop,
            name="voice-playback-worker",
            daemon=True,
        )
        self._playback_thread.start()

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
    def generation_id(self) -> int:
        with self._lock:
            return self._generation_id

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
        if not enabled:
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
        if self._state in (VoiceState.SPEAKING, VoiceState.TTS_PROCESSING):
            self.stop_speaking()

        self.start_listening()

    def start_listening(self) -> bool:
        with self._lock:
            self._turn_id += 1
            self._generation_id += 1
            self._tts_generation_id += 1
            session_id = self._session_id
            turn_id = self._turn_id

        self._clear_queues()
        self._set_state(VoiceState.LISTENING, "Đang nghe... Chạm lần nữa để gửi")
        success = self.capture.start()
        if not success:
            self._set_state(VoiceState.UNAVAILABLE, "Không tìm thấy microphone")
            threading.Timer(2.5, lambda: self._restore_ready_if_state(VoiceState.UNAVAILABLE)).start()
            return False
        return True

    def stop_listening(self) -> None:
        """Manually stop recording and begin STT processing."""
        self.capture.stop()

    def stop_speaking(self) -> None:
        """Stop current speech synthesis and mpv playback immediately without losing history."""
        with self._lock:
            self._tts_generation_id += 1
            if self._state in (VoiceState.TTS_PROCESSING, VoiceState.SPEAKING):
                self._state = VoiceState.READY
            sid = self._session_id
            tid = self._turn_id

        self._clear_queues()
        self.player.stop()
        self._playback_done_event.set()
        self._emit_state("Sẵn sàng trò chuyện")
        if self.on_tts_finished:
            try:
                self.on_tts_finished(sid, tid)
            except Exception as exc:
                LOG.warning("Error in on_tts_finished callback: %s", exc)

    def reset_session(self) -> None:
        """Invalidate all in-flight STT/TTS callbacks and abort playback/capture."""
        with self._lock:
            self._session_id += 1
            self._turn_id += 1
            self._generation_id += 1
            self._tts_generation_id += 1
            self._state = VoiceState.READY
        self.capture.cancel()
        self._clear_queues()
        self.player.stop()
        self._playback_done_event.set()
        self._emit_state("Sẵn sàng trò chuyện")

    def notify_llm_processing(self, session_id: int | None = None, turn_id: int | None = None) -> None:
        """Called when LLM begins thinking for a user query."""
        with self._lock:
            if session_id is not None:
                self._session_id = session_id
            if turn_id is not None:
                self._turn_id = turn_id
            self._generation_id += 1
            self._tts_generation_id += 1
            self._segment_index = 0
            self._segmenter = SentenceSegmenter(min_sentence_chars=6)
        self._clear_queues()
        self._set_state(VoiceState.LLM_PROCESSING, "Đang xử lý câu hỏi...")

    def handle_stream_delta(self, context: TurnContext, delta: str) -> None:
        """Process incremental LLM delta and enqueue completed sentences for TTS synthesis."""
        with self._lock:
            if (
                context.session_id != self._session_id
                or not self._speaker_enabled
                or not self.tts_enabled
            ):
                return
            if context.generation_id > 0 and context.generation_id != self._generation_id:
                return
            self._turn_id = context.turn_id
            tts_gen_id = self._tts_generation_id

        ctx = TurnContext(
            session_id=context.session_id,
            turn_id=context.turn_id,
            generation_id=context.generation_id,
            tts_generation_id=tts_gen_id,
        )

        completed_sentences = self._segmenter.push_delta(delta)
        for sentence in completed_sentences:
            with self._lock:
                idx = self._segment_index
                self._segment_index += 1
                if self._state == VoiceState.LLM_PROCESSING:
                    self._state = VoiceState.TTS_PROCESSING
                    self._emit_state("Đang tạo giọng nói...")
            try:
                self._tts_queue.put((ctx, idx, sentence, False), block=False)
            except queue.Full:
                LOG.warning("TTS sentence queue full, dropping sentence segment %d", idx)

    def handle_stream_completed(self, context: TurnContext, full_text: str) -> None:
        """Flush remaining text buffer as final sentence segment when LLM finishes."""
        with self._lock:
            if (
                context.session_id != self._session_id
                or not self._speaker_enabled
                or not self.tts_enabled
            ):
                return
            if context.generation_id > 0 and context.generation_id != self._generation_id:
                return
            self._turn_id = context.turn_id
            tts_gen_id = self._tts_generation_id

        ctx = TurnContext(
            session_id=context.session_id,
            turn_id=context.turn_id,
            generation_id=context.generation_id,
            tts_generation_id=tts_gen_id,
        )

        remaining_sentences = self._segmenter.flush()
        if not remaining_sentences and self._segment_index == 0:
            # Fallback if no sentences were emitted yet
            if full_text.strip():
                remaining_sentences = [full_text.strip()]

        if not remaining_sentences:
            # Nothing to speak
            try:
                self._tts_queue.put((ctx, self._segment_index, "", True), block=False)
            except queue.Full:
                pass
            return

        for i, sentence in enumerate(remaining_sentences):
            with self._lock:
                idx = self._segment_index
                self._segment_index += 1
                is_last = (i == len(remaining_sentences) - 1)
                if self._state == VoiceState.LLM_PROCESSING:
                    self._state = VoiceState.TTS_PROCESSING
                    self._emit_state("Đang tạo giọng nói...")
            try:
                self._tts_queue.put((ctx, idx, sentence, is_last), block=False)
            except queue.Full:
                LOG.warning("TTS sentence queue full, dropping final segment %d", idx)

    def handle_stream_interrupted(self, context: TurnContext) -> None:
        """Called when LLM generation was cancelled mid-stream."""
        self.stop_speaking()

    def handle_llm_response(self, session_id: int, text: str) -> None:
        """Full response fallback for non-streaming or direct text queries."""
        with self._lock:
            if session_id != self._session_id:
                LOG.info("Discarding stale LLM response from prior session.")
                return
            if not self._speaker_enabled or not self.tts_enabled:
                self._state = VoiceState.READY
                self._emit_state("Sẵn sàng trò chuyện")
                return
            turn_id = self._turn_id
            gen_id = self._generation_id
            tts_gen_id = self._tts_generation_id
            self._segment_index = 0
            self._segmenter = SentenceSegmenter(min_sentence_chars=6)

        ctx = TurnContext(
            session_id=session_id,
            turn_id=turn_id,
            generation_id=gen_id,
            tts_generation_id=tts_gen_id,
        )
        self.handle_stream_completed(ctx, text)

    def shutdown(self) -> None:
        """Gracefully release all audio hardware, threads, and subprocesses."""
        self._shutdown_event.set()
        self.capture.cancel()
        self.stop_speaking()
        self.player.shutdown()

    # ------------------------------------------------------------------
    # Internal Pipeline
    # ------------------------------------------------------------------

    def _clear_queues(self) -> None:
        while not self._tts_queue.empty():
            try:
                self._tts_queue.get_nowait()
            except queue.Empty:
                break
        while not self._playback_queue.empty():
            try:
                self._playback_queue.get_nowait()
            except queue.Empty:
                break

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
        if self.on_transcript_ready:
            self.on_transcript_ready(session_id, turn_id, transcript)

    # ------------------------------------------------------------------
    # Background Workers for Sequential Sentence TTS and Playback
    # ------------------------------------------------------------------

    def _tts_worker_loop(self) -> None:
        """Continuously pulls sentence text, calls Edge TTS via 9Router, and puts into playback queue."""
        while not self._shutdown_event.is_set():
            try:
                ctx, seg_idx, sentence, is_last = self._tts_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            with self._lock:
                if (
                    ctx.session_id != self._session_id
                    or ctx.tts_generation_id != self._tts_generation_id
                    or not self._speaker_enabled
                ):
                    continue

            audio_data = b""
            if sentence.strip():
                try:
                    voice_id, tts_text = resolve_voice_and_text(sentence)
                    audio_data = synthesize_speech_9router(
                        tts_text,
                        voice_id=voice_id,
                        base_url=self.tts_base_url,
                    )
                except Exception as exc:
                    LOG.warning("Sentence TTS synthesis failed for segment %d: %s", seg_idx, exc)
                    audio_data = b""

            with self._lock:
                if (
                    ctx.session_id != self._session_id
                    or ctx.tts_generation_id != self._tts_generation_id
                    or not self._speaker_enabled
                ):
                    continue

            self._playback_queue.put((ctx, seg_idx, audio_data, is_last))

    def _playback_worker_loop(self) -> None:
        """Pulls audio segments and plays them in strict sequential index order without overlapping."""
        buffered: dict[int, tuple[TurnContext, bytes, bool]] = {}
        expected_seg_idx = 0

        while not self._shutdown_event.is_set():
            try:
                ctx, seg_idx, audio_data, is_last = self._playback_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            with self._lock:
                if (
                    ctx.session_id != self._session_id
                    or ctx.tts_generation_id != self._tts_generation_id
                    or not self._speaker_enabled
                ):
                    buffered.clear()
                    expected_seg_idx = 0
                    continue

            buffered[seg_idx] = (ctx, audio_data, is_last)

            # Play segments strictly in sequential index order (0, 1, 2...)
            while expected_seg_idx in buffered:
                seg_ctx, seg_audio, seg_is_last = buffered.pop(expected_seg_idx)
                expected_seg_idx += 1

                with self._lock:
                    if (
                        seg_ctx.session_id != self._session_id
                        or seg_ctx.tts_generation_id != self._tts_generation_id
                        or not self._speaker_enabled
                    ):
                        buffered.clear()
                        expected_seg_idx = 0
                        break

                if seg_audio:
                    self._playback_done_event.clear()
                    played = self.player.play(seg_audio, request_id=seg_ctx.turn_id)
                    if played:
                        # Wait for this sentence to finish playing before starting next sentence
                        self._playback_done_event.wait(timeout=30.0)

                if seg_is_last:
                    buffered.clear()
                    expected_seg_idx = 0
                    with self._lock:
                        if self._state == VoiceState.SPEAKING:
                            self._state = VoiceState.READY
                            self._emit_state("Sẵn sàng trò chuyện")
                    if self.on_tts_finished:
                        self.on_tts_finished(seg_ctx.session_id, seg_ctx.turn_id)

    def _on_playback_started(self, request_id: int) -> None:
        """AudioPlayer actually began streaming to mpv."""
        LOG.info("Audio playback started (turn_id=%d)", request_id)
        with self._lock:
            self._state = VoiceState.SPEAKING
        self._emit_state("Souta đang nói...")
        if self.on_tts_started:
            self.on_tts_started(self._session_id, request_id)

    def _on_playback_finished(self, request_id: int) -> None:
        """mpv process finished playing current sentence."""
        LOG.info("Audio playback finished (turn_id=%d)", request_id)
        self._playback_done_event.set()

    def _on_playback_error(self, request_id: int, err_msg: str) -> None:
        LOG.error("Audio playback error (turn_id=%d): %s", request_id, err_msg)
        self._playback_done_event.set()
