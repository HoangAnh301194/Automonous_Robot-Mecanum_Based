"""Unit tests for Wake Word detection integration in Kiosk UI."""
from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from PyQt5.QtWidgets import QApplication

# Ensure QApplication exists for Qt tests
_APP = QApplication.instance() or QApplication([])

from robot_ui.kiosk.signals import KioskSignals
from robot_ui.kiosk.wakeword_worker import WakeWordWorker


class TestWakeWordWorker(unittest.TestCase):
    def setUp(self):
        self.signals = KioskSignals()

    def test_worker_disabled_via_env(self):
        with patch.dict("os.environ", {"ENABLE_WAKEWORD": "0"}):
            worker = WakeWordWorker(self.signals)
            self.assertFalse(worker.is_available())

    def test_pause_and_resume(self):
        worker = WakeWordWorker(self.signals)
        self.assertTrue(worker._paused)
        worker.resume_listening()
        # If not available, resume stays skipped
        if not worker.is_available():
            self.assertTrue(worker._paused)
        worker.pause_listening()
        self.assertTrue(worker._paused)

    def test_signal_emission(self):
        received = []
        self.signals.wake_word_detected.connect(lambda kw: received.append(kw))
        self.signals.wake_word_detected.emit("vi_sota_oi")
        self.assertEqual(received, ["vi_sota_oi"])


class TestKioskWindowWakeWordTransition(unittest.TestCase):
    def test_idle_to_chat_transition_on_wake_word(self):
        from robot_ui.kiosk.window import KioskWindow

        # Mock ROS node and audio subsystems
        mock_ros = MagicMock()
        mock_ros.get_logger.return_value = MagicMock()

        with patch("robot_ui.kiosk.window.VoiceController"), \
             patch("robot_ui.kiosk.window.LLMWorker"):
            signals = KioskSignals()
            win = KioskWindow(ros_node=mock_ros, signals=signals)
            win.show_page("idle")
            self.assertEqual(win._active_page, "idle")

            # Simulate wake word detected signal
            win.signals.wake_word_detected.emit("vi_sota_oi")

            # Must transition to chat page
            self.assertEqual(win._active_page, "chat")

            # Clean up
            win.close()


if __name__ == "__main__":
    unittest.main()
