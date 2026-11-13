"""Comprehensive offline unit tests for Wake Word Phase 1."""
from __future__ import annotations

import json
import tempfile
import unittest
import wave
from pathlib import Path

from robot_wakeword.download_models import get_models_from_config
from robot_wakeword.probe import (
    DEFAULT_CONFIG_PATH,
    get_active_keywords,
    load_aliases,
    load_config,
    load_models,
    match_wake_phrase,
    normalize,
    open_wav,
    recognize_file,
)


class WakeWordMatchingTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(DEFAULT_CONFIG_PATH)
        self.aliases = {key: item["aliases"] for key, item in cfg["keywords"].items()}

    def test_vietnamese_maica_aliases(self):
        for speech in ("Maica!", "Maica, giới thiệu về PTIT",
                       "mai ca ơi", "chào maica"):
            with self.subTest(speech=speech):
                self.assertTrue(match_wake_phrase(speech, "vi_maica", self.aliases))

    def test_reject_non_wake_phrases_and_word_boundaries(self):
        for speech in ("Mai là một cái tên", "maicabee", "my car",
                       "hôm nay thời tiết đẹp"):
            with self.subTest(speech=speech):
                self.assertFalse(match_wake_phrase(speech, "vi_maica", self.aliases))

    def test_unicode_normalization(self):
        self.assertEqual(normalize("  Ｍａｉｃａ！", "vi_maica"), "maica")

    def test_alias_overrides(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"vi_maica": ["maika"]}', encoding="utf-8")
            changed = load_aliases(str(path), self.aliases)
            self.assertTrue(match_wake_phrase("maika", "vi_maica", changed))
            self.assertFalse(match_wake_phrase("maica", "vi_maica", changed))

    def test_reject_invalid_alias_structure(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"unknown": ["foo"]}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_aliases(str(path), self.aliases)


class ConfigurationTests(unittest.TestCase):
    def test_only_vietnamese_configured(self):
        cfg = load_config(DEFAULT_CONFIG_PATH)
        self.assertEqual(list(cfg["keywords"]), ["vi_maica"])
        self.assertEqual(set(get_active_keywords(cfg)), {"vi_maica"})
        self.assertEqual(cfg["keywords"]["vi_maica"]["language"], "vi")

    def test_only_vietnamese_model_downloaded(self):
        names = get_models_from_config(DEFAULT_CONFIG_PATH)
        self.assertEqual(set(names), {"vosk-model-small-vn-0.4"})

    def test_rejects_extra_detector_configuration(self):
        cfg = load_config(DEFAULT_CONFIG_PATH)
        cfg["keywords"]["unknown_detector"] = {
            "language": "vi", "display_name": "Unknown",
            "model": "invalid-model", "aliases": ["unknown"], "enabled": True
        }
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", suffix=".json") as f:
            json.dump(cfg, f)
            f.flush()
            with self.assertRaises(ValueError):
                load_config(Path(f.name))

    def test_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            load_config(Path("/non/existent/path/keywords.json"))

    def test_malformed_json(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json") as f:
            f.write("{invalid_json:")
            f.flush()
            with self.assertRaises(ValueError):
                load_config(Path(f.name))

    def test_reject_invalid_schema(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json") as f:
            f.write(json.dumps({"keywords": {"vi_maica": {"language": "vi"}}}))
            f.flush()
            with self.assertRaises(ValueError):
                load_config(Path(f.name))

    def test_disabled_keyword_filtering(self):
        cfg = load_config(DEFAULT_CONFIG_PATH)
        cfg["keywords"]["vi_maica"]["enabled"] = False
        self.assertFalse(get_active_keywords(cfg))
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", suffix=".json") as f:
            json.dump(cfg, f)
            f.flush()
            self.assertEqual(get_models_from_config(Path(f.name)), {})


class DummyRecognizer:
    def __init__(self, model, rate):
        self.model = model
        self.calls = 0

    def AcceptWaveform(self, pcm):
        self.calls += 1
        return self.calls == 2

    def Result(self):
        return json.dumps({"text": "maica" if self.model == "vi" else ""})

    def FinalResult(self):
        return json.dumps({"text": "cho tôi hỏi" if self.model == "vi" else ""})


class DummyVosk:
    KaldiRecognizer = DummyRecognizer

    @staticmethod
    def SetLogLevel(level):
        pass


class AudioPipelineTests(unittest.TestCase):
    def _make_wav(self, path: Path, rate: int = 16000, channels: int = 1):
        with wave.open(str(path), "wb") as out:
            out.setnchannels(channels)
            out.setsampwidth(2)
            out.setframerate(rate)
            out.writeframes(b"\x00\x00" * rate)

    def test_recognize_file(self):
        with tempfile.TemporaryDirectory() as d:
            wav = Path(d) / "sample.wav"
            self._make_wav(wav)
            transcripts, elapsed, duration = recognize_file(
                wav, DummyVosk, {"vi_maica": "vi"}
            )
            self.assertEqual(transcripts, {"vi_maica": "maica cho tôi hỏi"})
            self.assertAlmostEqual(duration, 1.0)
            self.assertGreaterEqual(elapsed, 0)

    def test_rejects_incompatible_wav(self):
        with tempfile.TemporaryDirectory() as d:
            wav = Path(d) / "bad.wav"
            self._make_wav(wav, rate=8000)
            with self.assertRaises(ValueError):
                with open_wav(wav):
                    pass

    def test_missing_model_directory_exits(self):
        with tempfile.TemporaryDirectory() as d:
            active = {"vi_maica": {"model": "non_existent_folder"}}
            with self.assertRaises(SystemExit):
                load_models(Path(d), active)


if __name__ == "__main__":
    unittest.main()
