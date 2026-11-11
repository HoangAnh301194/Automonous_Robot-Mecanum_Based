"""Unit and integration tests for LLM SSE Streaming and Sentence-by-Sentence TTS."""
from __future__ import annotations

import io
import json
import threading
import time
from unittest.mock import MagicMock, patch

import pytest
from robot_ui.kiosk.llm_client import LLMWorker, TurnContext, stream_llm
from robot_ui.kiosk.speech_client import SentenceSegmenter
from robot_ui.kiosk.audio_worker import VoiceController, VoiceState


# ----------------------------------------------------------------------
# F2. Sentence Segmenter Tests
# ----------------------------------------------------------------------

def test_sentence_segmenter_basic():
    segmenter = SentenceSegmenter(min_sentence_chars=5)
    # Stream in small deltas
    out = segmenter.push_delta("Xin chào, ")
    assert out == []
    out = segmenter.push_delta("tôi là Souta. ")
    assert out == ["Xin chào, tôi là Souta."]
    out = segmenter.push_delta("Tôi có thể giúp gì?")
    assert out == []
    # Flush at the end of LLM stream
    out = segmenter.flush()
    assert out == ["Tôi có thể giúp gì?"]


def test_sentence_segmenter_titles_and_decimals():
    segmenter = SentenceSegmenter(min_sentence_chars=5)
    deltas = [
        "TS. ", "Hoàng Sơn ", "là giảng viên ", "tại PTIT. ",
        "Nhiệt độ phòng là ", "24.5 độ C. ",
        "Thầy nghiên cứu ", "robotics, v.v."
    ]
    extracted = []
    for d in deltas:
        extracted.extend(segmenter.push_delta(d))
    extracted.extend(segmenter.flush())

    assert len(extracted) == 3
    # TS. was not broken prematurely
    assert extracted[0] == "TS. Hoàng Sơn là giảng viên tại PTIT."
    # Decimal 24.5 was not broken prematurely
    assert extracted[1] == "Nhiệt độ phòng là 24.5 độ C."
    assert "robotics, v.v." in extracted[2]


def test_sentence_segmenter_asian_punctuations():
    segmenter = SentenceSegmenter(min_sentence_chars=3)
    text = "こんにちは。私はソウタです！何かご用でしょうか？"
    out = segmenter.push_delta(text)
    out.extend(segmenter.flush())
    assert len(out) == 3
    assert out[0] == "こんにちは。"
    assert out[1] == "私はソウタです！"
    assert out[2] == "何かご用でしょうか？"


# ----------------------------------------------------------------------
# F2. SSE Parser and UTF-8 Chunking Tests
# ----------------------------------------------------------------------

class MockSSEResponse:
    def __init__(self, raw_chunks: list[bytes], headers=None):
        self._chunks = list(raw_chunks)
        self.headers = headers or {"Content-Type": "text/event-stream"}

    def read(self, n=256):
        if self._chunks:
            return self._chunks.pop(0)
        return b""

    def close(self):
        pass


def test_stream_llm_sse_normal_and_utf8_split():
    # Split a 3-byte UTF-8 character across chunk boundary
    # 'ế' in UTF-8 is b'\xe1\xba\xbf'
    e_hat_byte1 = b"\xe1"
    e_hat_byte23 = b"\xba\xbf"

    event1 = b'data: {"choices":[{"delta":{"content":"Ti' + e_hat_byte1
    event2 = e_hat_byte23 + b'n"}}]}\n\n'
    event3 = 'data: {"choices":[{"delta":{"content":" s\u0129"}}]}\n\ndata: [DONE]\n\n'.encode("utf-8")

    mock_resp = MockSSEResponse([event1, event2, event3])

    with patch("urllib.request.urlopen", return_value=mock_resp):
        deltas = list(stream_llm("test", api_key="sk-test"))

    full = "".join(deltas)
    assert "Tiến" in full
    assert "sĩ" in full


def test_stream_llm_cancellation():
    cancel_evt = threading.Event()
    # Continuous events
    chunks = [
        b'data: {"choices":[{"delta":{"content":"Part 1"}}]}\n\n',
        b'data: {"choices":[{"delta":{"content":"Part 2"}}]}\n\n',
        b'data: {"choices":[{"delta":{"content":"Part 3"}}]}\n\n',
    ]
    mock_resp = MockSSEResponse(chunks)

    collected = []
    with patch("urllib.request.urlopen", return_value=mock_resp):
        gen = stream_llm("test", api_key="sk-test", cancel_event=cancel_evt)
        collected.append(next(gen))
        # Trigger cancel
        cancel_evt.set()
        # Next should stop
        remaining = list(gen)

    assert len(collected) == 1
    assert collected[0] == "Part 1"
    assert len(remaining) == 0


# ----------------------------------------------------------------------
# F1. LLMWorker Streaming & Reliability (P0)
# ----------------------------------------------------------------------

def test_llm_worker_streaming_and_history_isolation():
    deltas = []
    completed = []

    def on_delta(ctx: TurnContext, text: str):
        deltas.append((ctx.turn_id, text))

    def on_completed(ctx: TurnContext, text: str):
        completed.append((ctx.turn_id, text))

    worker = LLMWorker(
        on_response=lambda s, t: None,
        on_error=lambda s, e: None,
        on_stream_delta=on_delta,
        on_stream_completed=on_completed,
        api_key="test-key",
    )

    mock_chunks = ["Xin ", "chào ", "bạn."]
    with patch("robot_ui.kiosk.llm_client.stream_llm", return_value=iter(mock_chunks)):
        assert worker.send("Chào robot")
        # Wait for worker thread
        time.sleep(0.1)

    assert len(deltas) == 3
    assert len(completed) == 1
    assert completed[0][1] == "Xin chào bạn."

    # History contains clean turn
    with worker._lock:
        hist = worker._history
        assert len(hist) == 2
        assert hist[0]["content"] == "Chào robot"
        assert hist[1]["content"] == "Xin chào bạn."


def test_llm_worker_cancel_preserves_past_history():
    interrupted = []

    def on_interrupted(ctx: TurnContext):
        interrupted.append(ctx.turn_id)

    worker = LLMWorker(
        on_response=lambda s, t: None,
        on_error=lambda s, e: None,
        on_stream_interrupted=on_interrupted,
        api_key="test-key",
    )

    # Pretend turn 1 completed cleanly
    worker._history = [
        {"role": "user", "content": "Question 1"},
        {"role": "assistant", "content": "Answer 1"},
    ]

    # Start turn 2
    started = threading.Event()
    release = threading.Event()

    def slow_stream(*args, **kwargs):
        started.set()
        release.wait(2.0)
        yield "Delayed token"

    with patch("robot_ui.kiosk.llm_client.stream_llm", slow_stream):
        assert worker.send("Question 2")
        assert started.wait(1.0)
        assert worker.is_busy

        # Cancel turn 2 mid-stream
        worker.cancel_current_request()
        release.set()
        time.sleep(0.1)

    assert not worker.is_busy
    assert len(interrupted) == 1

    # Crucial P0: Completed history from turn 1 is NOT wiped!
    with worker._lock:
        assert len(worker._history) == 2
        assert worker._history[0]["content"] == "Question 1"
        assert worker._history[1]["content"] == "Answer 1"


# ----------------------------------------------------------------------
# F1. VoiceController Sentence Streaming & Cancellation Tests
# ----------------------------------------------------------------------

def test_voice_controller_sentence_streaming_and_stop():
    started_events = []
    finished_events = []

    vc = VoiceController(
        on_tts_started=lambda s, t: started_events.append((s, t)),
        on_tts_finished=lambda s, t: finished_events.append((s, t)),
    )

    # Mock synthesize_speech_9router to return fake WAV bytes
    fake_wav = b"RIFF....WAVEfmt ...."
    with patch("robot_ui.kiosk.audio_worker.synthesize_speech_9router", return_value=fake_wav):
        with patch.object(vc.player, "play", return_value=True):
            vc.notify_llm_processing()
            assert vc.state == VoiceState.LLM_PROCESSING

            ctx = TurnContext(
                session_id=vc.session_id,
                turn_id=vc.turn_id,
                generation_id=vc._generation_id,
                tts_generation_id=vc._tts_generation_id,
            )

            # Feed deltas
            vc.handle_stream_delta(ctx, "Xin chào bạn. ")
            time.sleep(0.1)

            # When TTS is synthesizing or queued
            assert vc.state in (VoiceState.TTS_PROCESSING, VoiceState.SPEAKING)

            # Stop speaking immediately
            vc.stop_speaking()
            assert vc.state == VoiceState.READY
            assert len(finished_events) >= 1

    vc.shutdown()


def test_voice_controller_speaker_off_clears_queue():
    vc = VoiceController()
    vc.set_speaker_enabled(False)
    assert not vc.is_speaker_enabled

    ctx = TurnContext(
        session_id=vc.session_id,
        turn_id=vc.turn_id,
        generation_id=vc._generation_id,
        tts_generation_id=vc._tts_generation_id,
    )
    # Stream events while speaker is disabled
    vc.handle_stream_delta(ctx, "Không phát âm thanh khi đã tắt loa. ")
    time.sleep(0.05)

    # No items in TTS queue
    assert vc._tts_queue.empty()
    assert vc.state == VoiceState.READY

    vc.shutdown()


def test_llm_worker_does_not_duplicate_response_callback():
    response_called = []
    completed_called = []

    worker = LLMWorker(
        on_response=lambda s, t: response_called.append(t),
        on_error=lambda s, e: None,
        on_stream_completed=lambda ctx, t: completed_called.append(t),
        api_key="test-key",
    )

    with patch("robot_ui.kiosk.llm_client.stream_llm", return_value=iter(["Chào bạn."])):
        assert worker.send("Chào")
        time.sleep(0.1)

    assert len(completed_called) == 1
    assert completed_called[0] == "Chào bạn."
    assert len(response_called) == 0  # No duplicate callback when stream_completed is provided

