"""Audio capture module using parec (PulseAudio) with RMS VAD and silence timeout."""
from __future__ import annotations

import io
import logging
import math
import os
import struct
import subprocess
import threading
import time
import wave
from typing import Callable

from robot_ui.kiosk.camera_mic_resolver import get_effective_mic_source

LOG = logging.getLogger(__name__)


def pcm_to_wav(pcm_bytes: bytes, sample_rate: int = 16000, channels: int = 1) -> bytes:
    """Wrap raw 16-bit PCM bytes into a standard WAV container in memory."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(pcm_bytes)
    return buf.getvalue()


def calculate_rms(pcm_chunk: bytes) -> float:
    """Calculate Root Mean Square (RMS) energy of a 16-bit mono PCM chunk."""
    if not pcm_chunk or len(pcm_chunk) < 2:
        return 0.0
    num_samples = len(pcm_chunk) // 2
    # Truncate to even number of bytes for 16-bit samples
    even_bytes = pcm_chunk[: num_samples * 2]
    try:
        samples = struct.unpack(f"<{num_samples}h", even_bytes)
    except struct.error:
        return 0.0
    if not samples:
        return 0.0
    sum_squares = sum(s * s for s in samples)
    return math.sqrt(sum_squares / len(samples))


class AudioCapture:
    """Microphone audio capture runner managed in a background thread."""

    def __init__(
        self,
        on_audio_ready: Callable[[bytes, float], None] | None = None,
        on_no_speech: Callable[[], None] | None = None,
        on_error: Callable[[str], None] | None = None,
        *,
        sample_rate: int = 16000,
        channels: int = 1,
        max_duration_seconds: float = 15.0,
        min_duration_seconds: float = 0.3,
        silence_timeout_seconds: float = 1.5,
        rms_threshold: float = 200.0,
        source_name: str | None = None,
    ) -> None:
        self.on_audio_ready = on_audio_ready  # (wav_bytes, duration_seconds)
        self.on_no_speech = on_no_speech
        self.on_error = on_error

        self.sample_rate = int(os.getenv("AUDIO_SAMPLE_RATE", str(sample_rate)))
        self.channels = int(os.getenv("AUDIO_CHANNELS", str(channels)))
        self.max_duration = float(os.getenv("AUDIO_MAX_RECORD_SECONDS", str(max_duration_seconds)))
        self.min_duration = float(os.getenv("AUDIO_MIN_RECORD_SECONDS", str(min_duration_seconds)))
        self.silence_timeout = float(os.getenv("AUDIO_SILENCE_TIMEOUT_SECONDS", str(silence_timeout_seconds)))
        self.rms_threshold = float(os.getenv("AUDIO_RMS_THRESHOLD", str(rms_threshold)))
        self.source_name = get_effective_mic_source(source_name)

        self._process: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._is_recording = False

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._is_recording

    def start(self) -> bool:
        """Start recording from the microphone in a background thread."""
        with self._lock:
            if self._is_recording:
                LOG.warning("AudioCapture already recording.")
                return False
            self._is_recording = True
            self._stop_event.clear()

        source = self.source_name or "@DEFAULT_SOURCE@"
        cmd = [
            "parec",
            "-d",
            source,
            f"--rate={self.sample_rate}",
            "--format=s16le",
            f"--channels={self.channels}",
            "--latency-msec=50",
            "--raw",
        ]
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception as exc:
            with self._lock:
                self._is_recording = False
            LOG.exception("Failed to start parec process: %s", exc)
            if self.on_error:
                self.on_error(f"Không thể khởi động microphone ({exc})")
            return False

        self._thread = threading.Thread(target=self._record_loop, name="audio-capture-thread", daemon=True)
        self._thread.start()
        LOG.info("Audio capture started on source: %s", source)
        return True

    def stop(self) -> None:
        """Request the capture loop to stop and finalize audio."""
        with self._lock:
            if not self._is_recording:
                return
            self._is_recording = False
            self._stop_event.set()

        proc = self._process
        if proc and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass

    def cancel(self) -> None:
        """Abort recording immediately and discard all buffered audio."""
        with self._lock:
            self._is_recording = False
            self._stop_event.set()

        proc = self._process
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=0.5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _record_loop(self) -> None:
        proc = self._process
        if not proc or not proc.stdout:
            return

        chunk_bytes = int(self.sample_rate * 2 * 0.05)  # 50ms chunk (1600 bytes)
        frames: list[bytes] = []
        start_time = time.monotonic()
        speech_detected = False
        last_speech_time = start_time

        try:
            while not self._stop_event.is_set():
                chunk = proc.stdout.read(chunk_bytes)
                if not chunk:
                    if proc.poll() is not None:
                        break
                    time.sleep(0.01)
                    continue

                frames.append(chunk)
                now = time.monotonic()
                total_duration = now - start_time

                rms = calculate_rms(chunk)
                if rms >= self.rms_threshold:
                    speech_detected = True
                    last_speech_time = now
                elif speech_detected:
                    silence_duration = now - last_speech_time
                    if silence_duration >= self.silence_timeout and total_duration >= self.min_duration:
                        LOG.info(
                            "Silence timeout reached (%.2fs silence, total %.2fs)",
                            silence_duration,
                            total_duration,
                        )
                        break

                if total_duration >= self.max_duration:
                    LOG.info("Max record duration reached (%.2fs)", total_duration)
                    break
        except Exception as exc:
            LOG.exception("Error in audio capture loop: %s", exc)
            if self.on_error:
                self.on_error("Lỗi khi đọc âm thanh từ microphone.")
            return
        finally:
            with self._lock:
                self._is_recording = False
            if proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=0.3)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

        total_pcm = b"".join(frames)
        duration = len(total_pcm) / (self.sample_rate * 2) if self.sample_rate else 0.0

        if not speech_detected or duration < self.min_duration:
            LOG.info("No speech detected or duration too short (%.2fs)", duration)
            if self.on_no_speech:
                self.on_no_speech()
            return

        wav_data = pcm_to_wav(total_pcm, self.sample_rate, self.channels)
        LOG.info("Captured valid speech: %.2fs (%d bytes WAV)", duration, len(wav_data))
        if self.on_audio_ready:
            self.on_audio_ready(wav_data, duration)
