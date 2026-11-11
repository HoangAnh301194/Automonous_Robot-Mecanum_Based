"""Background worker for wake word detection in Kiosk UI."""
from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
import time
from pathlib import Path
from PyQt5.QtCore import QThread

from robot_ui.kiosk.signals import KioskSignals

LOG = logging.getLogger(__name__)


class WakeWordWorker(QThread):
    """Monitors wake word events and notifies Kiosk UI."""

    def __init__(self, signals: KioskSignals, parent=None) -> None:
        super().__init__(parent)
        self.signals = signals
        self._enabled = os.getenv("ENABLE_WAKEWORD", "1").lower() not in {"0", "false", "no"}
        self._paused = True
        self._stop_event = threading.Event()
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

        # Locate Python environment & probe script
        self._venv_python = Path.home() / ".venvs" / "souta-wakeword" / "bin" / "python"
        self._probe_script = Path(__file__).resolve().parents[1] / "wakeupword" / "probe.py"
        self._models_dir = Path.home() / "vosk-models"

    def is_available(self) -> bool:
        """Check if environment and models are ready to run."""
        python_bin = self._venv_python if self._venv_python.is_file() else Path("/usr/bin/python3")
        return (
            self._enabled
            and self._probe_script.is_file()
            and self._models_dir.is_dir()
            and any(self._models_dir.iterdir())
        )

    def pause_listening(self) -> None:
        """Pause wake word detection to release microphone."""
        with self._lock:
            self._paused = True
            if self._process and self._process.poll() is None:
                LOG.info("[WakeWord] Pausing wake word listener (releasing microphone)")
                try:
                    self._process.terminate()
                    self._process.wait(timeout=1.0)
                except (subprocess.TimeoutExpired, OSError):
                    try:
                        self._process.kill()
                    except OSError:
                        pass
                self._process = None

    def resume_listening(self) -> None:
        """Resume wake word detection when on idle screen."""
        if not self.is_available():
            LOG.debug("[WakeWord] Wake word not available or disabled; skipping resume")
            return
        with self._lock:
            self._paused = False

    def stop(self) -> None:
        """Terminate the worker completely."""
        self._stop_event.set()
        self.pause_listening()
        self.wait(1500)

    def run(self) -> None:
        """Main monitoring loop in QThread."""
        if not self.is_available():
            LOG.info("[WakeWord] Wake word environment not configured. Listener standby.")

        while not self._stop_event.is_set():
            if self._paused or not self.is_available():
                time.sleep(0.5)
                continue

            # Start probe process
            python_bin = str(self._venv_python) if self._venv_python.is_file() else "python3"
            cmd = [
                python_bin,
                str(self._probe_script),
                "live",
                "--models-dir",
                str(self._models_dir),
            ]
            LOG.info("[WakeWord] Starting live wake word process: %s", " ".join(cmd))
            try:
                with self._lock:
                    if self._paused:
                        continue
                    self._process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        text=True,
                        bufsize=1,
                    )

                while not self._stop_event.is_set() and not self._paused:
                    if not self._process or self._process.poll() is not None:
                        break
                    line = self._process.stdout.readline()
                    if not line:
                        continue
                    line = line.strip()
                    if not line.startswith("{"):
                        continue
                    try:
                        data = json.loads(line)
                        if data.get("event") == "wake_detected":
                            keyword = data.get("keyword", "")
                            LOG.info("[WakeWord] Detected wake word: %s", keyword)
                            self.signals.wake_word_detected.emit(keyword)
                    except json.JSONDecodeError:
                        pass

            except Exception as exc:
                LOG.warning("[WakeWord] Subprocess error: %s", exc)
                time.sleep(2.0)
            finally:
                with self._lock:
                    if self._process:
                        try:
                            self._process.terminate()
                        except OSError:
                            pass
                        self._process = None

            time.sleep(0.5)
