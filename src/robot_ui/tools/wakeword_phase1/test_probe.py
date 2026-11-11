"""Offline unit tests: no ROS, hardware, network, or Vosk install required."""
import json
import tempfile
import unittest
import wave
from pathlib import Path

from probe import ALIASES, load_aliases, match_wake_phrase, normalize, open_wav, recognize_file


class WakeWordMatchingTests(unittest.TestCase):
    def test_vietnamese_trigger(self):
        self.assertTrue(match_wake_phrase("Sota ơi, giới thiệu về PTIT", "vi_sota_oi"))
        self.assertTrue(match_wake_phrase("Souta ơi!", "vi_sota_oi"))
        self.assertFalse(match_wake_phrase("Sota mới là một cái tên", "vi_sota_oi"))

    def test_english_trigger_and_boundaries(self):
        self.assertTrue(match_wake_phrase("Hey Souta! How are you?", "en_hey_souta"))
        self.assertTrue(match_wake_phrase("hey sota", "en_hey_souta"))
        self.assertFalse(match_wake_phrase("hey soutabee", "en_hey_souta"))
        self.assertFalse(match_wake_phrase("hey somebody", "en_hey_souta"))

    def test_japanese_variants(self):
        self.assertTrue(match_wake_phrase("そうたくん、こんにちは", "ja_soutakun"))
        self.assertTrue(match_wake_phrase("そうた君", "ja_soutakun"))
        self.assertTrue(match_wake_phrase("ソウタクン", "ja_soutakun"))
        self.assertFalse(match_wake_phrase("そうだね", "ja_soutakun"))

    def test_normalization(self):
        self.assertEqual(normalize("  Ｈｅｙ　Ｓｏｕｔａ！", "en_hey_souta"), "hey souta")
        self.assertEqual(normalize("ソウタクン", "ja_soutakun"), "そうたくん")
        self.assertIn("vi_sota_oi", ALIASES)

    def test_alias_overrides_are_explicit(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"vi_sota_oi": ["sô ta ơi"]}', encoding="utf-8")
            changed = load_aliases(str(path))
            self.assertTrue(match_wake_phrase("sô ta ơi", "vi_sota_oi", changed))
            self.assertFalse(match_wake_phrase("sota ơi", "vi_sota_oi", changed))
            self.assertEqual(changed["ja_soutakun"], ALIASES["ja_soutakun"])

    def test_rejects_invalid_alias_structure(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"unknown": ["foo"]}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_aliases(str(path))


class DummyRecognizer:
    def __init__(self, model, rate):
        self.model = model
        self.calls = 0
        self.finished = False

    def AcceptWaveform(self, pcm):
        self.calls += 1
        if self.calls == 2:
            self.finished = True
            return True
        return False

    def Result(self):
        return json.dumps({"text": "hey souta" if self.model == "en" else ""})

    def FinalResult(self):
        return json.dumps({"text": "please" if self.model == "en" else ""})


class DummyVosk:
    KaldiRecognizer = DummyRecognizer


class AudioPipelineTests(unittest.TestCase):
    def _make_wav(self, path, rate=16000, channels=1):
        with wave.open(str(path), "wb") as out:
            out.setnchannels(channels)
            out.setsampwidth(2)
            out.setframerate(rate)
            out.writeframes(b"\x00\x00" * rate)

    def test_recognize_file_keeps_completed_segments(self):
        with tempfile.TemporaryDirectory() as d:
            wav = Path(d) / "sample.wav"
            self._make_wav(wav)
            transcripts, elapsed, duration = recognize_file(
                wav, DummyVosk, {"en_hey_souta": "en", "vi_sota_oi": "vi"})
            self.assertEqual(transcripts["en_hey_souta"], "hey souta please")
            self.assertEqual(transcripts["vi_sota_oi"], "")
            self.assertAlmostEqual(duration, 1.0)
            self.assertGreaterEqual(elapsed, 0)

    def test_rejects_incompatible_audio(self):
        with tempfile.TemporaryDirectory() as d:
            wav = Path(d) / "bad.wav"
            self._make_wav(wav, rate=8000)
            with self.assertRaises(ValueError):
                with open_wav(wav):
                    pass


if __name__ == "__main__":
    unittest.main()
