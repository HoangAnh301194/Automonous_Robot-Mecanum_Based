"""Headless tests for Souta KWS recording workflow (Tk display not required)."""
from __future__ import annotations

import json
import tempfile
import unittest
import wave
from pathlib import Path

from recorder_gui import RecorderStore, SAMPLE_RATE, write_preview


class RecorderStoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.store = RecorderStore(self.root / "data")
        self.voice = (b"\x10\x27" * SAMPLE_RATE)  # 1 s audible S16 PCM

    def test_registration_persists_pseudonyms_and_unique_names(self):
        a = self.store.register("speakers", "  Hoàng   Anh ")
        b = self.store.register("speakers", "Linh")
        self.assertEqual(a, {"name": "Hoàng Anh", "id": "speaker_001"})
        self.assertEqual(b["id"], "speaker_002")
        with self.assertRaisesRegex(ValueError, "đã được đăng ký"):
            self.store.register("speakers", "hoàng anh")
        reloaded = RecorderStore(self.store.data_dir)
        self.assertEqual(reloaded.members("speakers"), self.store.members("speakers"))
        registry_json = (self.store.data_dir / "recorder_registry.json").read_text(encoding="utf-8")
        self.assertEqual(json.loads(registry_json)["speakers"][0]["name"], "Hoàng Anh")

    def test_preview_is_not_a_saved_training_file(self):
        member = self.store.register("speakers", "Người thử")
        preview = write_preview(self.store.data_dir, self.voice)
        self.assertTrue(preview.is_file())
        self.assertEqual(self.store.count("positive", member["id"]), 0)
        preview.unlink()
        self.assertEqual(self.store.count("positive", member["id"]), 0)

    def test_save_adds_incrementing_files_without_overwriting(self):
        member = self.store.register("speakers", "Người thử")
        preview = write_preview(self.store.data_dir, self.voice)
        dest1 = self.store.save_preview(preview, "positive", member["id"])
        dest2 = self.store.save_preview(preview, "positive", member["id"])
        self.assertEqual([dest1.name, dest2.name], ["001.wav", "002.wav"])
        self.assertEqual(self.store.count("positive", member["id"]), 2)
        with wave.open(str(dest1), "rb") as wav:
            self.assertEqual((wav.getnchannels(), wav.getsampwidth(), wav.getframerate()), (1, 2, SAMPLE_RATE))
        preview.unlink()

    def test_shared_speaker_id_for_multiple_categories(self):
        member = self.store.register("speakers", "Người thử")
        preview = write_preview(self.store.data_dir, self.voice)
        pos = self.store.save_preview(preview, "positive", member["id"])
        neg = self.store.save_preview(preview, "negative", member["id"])
        self.assertEqual(pos.parent.name, neg.parent.name)
        self.assertEqual(pos.parent.name, "speaker_001")
        preview.unlink()

    def test_ambient_sessions_are_independent(self):
        session = self.store.register("sessions", "Quạt phòng nghiên cứu")
        preview = write_preview(self.store.data_dir, self.voice)
        dest = self.store.save_preview(preview, "ambient", session["id"])
        self.assertEqual(dest.parent.name, "session_001")
        with self.assertRaises(ValueError):
            self.store.save_preview(preview, "positive", session["id"])
        preview.unlink()

    def test_reject_invalid_or_silent_preview(self):
        member = self.store.register("speakers", "Người thử")
        bad = write_preview(self.store.data_dir, b"\x00\x00" * SAMPLE_RATE)
        with self.assertRaisesRegex(ValueError, "trống"):
            self.store.save_preview(bad, "positive", member["id"])
        with self.assertRaises(ValueError):
            self.store.save_preview(bad, "arbitrary_label", member["id"])
        self.assertEqual(self.store.count("positive", member["id"]), 0)
        bad.unlink()


if __name__ == "__main__":
    unittest.main()
