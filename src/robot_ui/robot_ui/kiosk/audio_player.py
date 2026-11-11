"""Audio player using dedicated mpv subprocess with PID management and clean stops."""
from __future__ import annotations

import logging
import os
import subprocess
import threading
from typing import Callable

LOG = logging.getLogger(__name__)


class AudioPlayer:
    """Manages audio playback via mpv subprocess with thread safety and instant abort."""

    def __init__(
        self,
        on_playback_started: Callable[[int], None] | None = None,
        on_playback_finished: Callable[[int], None] | None = None,
        on_playback_error: Callable[[int, str], None] | None = None,
        player_cmd: str | None = None,
    ) -> None:
        self.on_playback_started = on_playback_started    # (request_id)
        self.on_playback_finished = on_playback_finished  # (request_id)
        self.on_playback_error = on_playback_error        # (request_id, error_msg)

        self.player_cmd = player_cmd or os.getenv("KIOSK_AUDIO_PLAYER", "mpv")
        self._muted = False
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._worker_thread: threading.Thread | None = None
        self._active_request_id: int | None = None

    @property
    def is_muted(self) -> bool:
        with self._lock:
            return self._muted

    def set_muted(self, muted: bool) -> None:
        """Toggle mute. If muted while playing, stops current playback immediately."""
        with self._lock:
            self._muted = muted
        if muted:
            self.stop()

    def is_playing(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def play(self, audio_data: bytes, request_id: int = 0) -> bool:
        """
        Play audio data asynchronously.
        If another playback is in progress, it is stopped first.
        Returns False if muted or data is empty.
        """
        if not audio_data:
            return False

        with self._lock:
            if self._muted:
                LOG.info("AudioPlayer is muted; skipping playback (request_id=%d)", request_id)
                return False

        # Stop existing playback cleanly first
        self.stop()

        cmd = [self.player_cmd, "--no-terminal", "--idle=no", "-"]
        try:
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            LOG.exception("Failed to launch %s: %s", self.player_cmd, exc)
            if self.on_playback_error:
                self.on_playback_error(request_id, str(exc))
            return False

        with self._lock:
            self._process = proc
            self._active_request_id = request_id

        self._worker_thread = threading.Thread(
            target=self._stream_worker,
            args=(proc, audio_data, request_id),
            name=f"audio-player-{request_id}",
            daemon=True,
        )
        self._worker_thread.start()
        return True

    def stop(self) -> None:
        """Stop current playback immediately by killing its specific subprocess."""
        proc = None
        with self._lock:
            proc = self._process
            self._process = None
            self._active_request_id = None

        if proc and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=0.3)
            except Exception:
                try:
                    proc.kill()
                    proc.wait(timeout=0.2)
                except Exception:
                    pass

    def shutdown(self) -> None:
        """Clean up resources on kiosk shutdown."""
        self.stop()

    def _stream_worker(self, proc: subprocess.Popen, audio_data: bytes, request_id: int) -> None:
        try:
            if self.on_playback_started:
                self.on_playback_started(request_id)

            # Stream audio into stdin
            if proc.stdin:
                try:
                    proc.stdin.write(audio_data)
                    proc.stdin.flush()
                    proc.stdin.close()
                except (BrokenPipeError, OSError):
                    pass

            proc.wait()
        except Exception as exc:
            LOG.exception("Error during audio streaming: %s", exc)
            if self.on_playback_error:
                self.on_playback_error(request_id, str(exc))
            return
        finally:
            with self._lock:
                if self._process == proc:
                    self._process = None
                is_active = (self._active_request_id == request_id)
                if is_active:
                    self._active_request_id = None

            if is_active and self.on_playback_finished:
                self.on_playback_finished(request_id)
