"""Audio player using dedicated mpv subprocess with PID management and clean stops."""
from __future__ import annotations

import logging
import os
import re
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
        self.playback_speed = os.getenv("AUDIO_PLAYBACK_SPEED", "1.15").strip()
        try:
            default_vol = int(os.getenv("AUDIO_PLAYBACK_VOLUME", "80"))
        except ValueError:
            default_vol = 80
        self._volume = max(0, min(100, default_vol))
        self._muted = False
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._worker_thread: threading.Thread | None = None
        self._active_request_id: int | None = None

    @property
    def volume(self) -> int:
        with self._lock:
            return self._volume

    def set_volume(self, volume: int) -> None:
        """Set volume in percent (0..100). Applies to future playback and default sink."""
        val = max(0, min(100, int(volume)))
        with self._lock:
            self._volume = val
        threading.Thread(
            target=self._apply_system_volume,
            args=(val,),
            name="pactl-set-volume",
            daemon=True,
        ).start()

    @staticmethod
    def _apply_system_volume(volume: int) -> None:
        try:
            subprocess.run(
                ["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{volume}%"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                check=False,
            )
        except Exception as exc:
            LOG.debug("Failed to set system volume via pactl: %s", exc)

    @staticmethod
    def get_system_volume() -> int:
        """Query current volume of default sink via pactl, default to 80 if failed."""
        try:
            res = subprocess.run(
                ["pactl", "get-sink-volume", "@DEFAULT_SINK@"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
                check=False,
            )
            match = re.search(r"(\d+)%", res.stdout)
            if match:
                return max(0, min(100, int(match.group(1))))
        except Exception:
            pass
        return 80

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

        vol = self._volume
        cmd = [self.player_cmd, "--no-terminal", "--idle=no", f"--speed={self.playback_speed}", f"--volume={vol}", "-"]
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
