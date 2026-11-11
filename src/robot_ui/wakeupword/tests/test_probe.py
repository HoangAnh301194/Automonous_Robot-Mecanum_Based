"""Comprehensive offline unit tests for Wake Word Phase 1."""
from __future__ import annotations

import argparse
import json
import tempfile
import unittest
import wave
from pathlib import Path

from probe import (
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
        self.config = load_config(DEFAULT_CONFIG_PATH)
        self.aliases = {k: v["aliases"] for k, v in self.config["keywords"].items()}

    def test_vietnamese_trigger(self):
        self.assertTrue(match_wake_phrase("Maica, giới thiệu về PTIT", "vi_maica", self.aliases))
        self.assertTrue(match_wake_phrase("Maica!", "vi_maica", self.aliases))
        self.assertFalse(match_wake_phrase("Mai là một cái tên", "vi_maica", self.aliases))

    def test_english_trigger_and_boundaries(self):
        self.assertTrue(match_wake_phrase("Maica! How are you?", "en_maica", self.aliases))
        self.assertTrue(match_wake_phrase("maica", "en_maica", self.aliases))
        self.assertFalse(match_wake_phrase("maicabee", "en_maica", self.aliases))
        self.assertFalse(match_wake_phrase("hey somebody", "en_maica", self.aliases))

    def test_japanese_variants_and_kana(self):
        self.assertTrue(match_wake_phrase("まいか、こんにちは", "ja_maica", self.aliases))
        self.assertTrue(match_wake_phrase("マイカ", "ja_maica", self.aliases))
        self.assertTrue(match_wake_phrase("マイカ", "ja_maica", self.aliases))
        self.assertFalse(match_wake_phrase("そうだね", "ja_maica", self.aliases))

    def test_unicode_normalization(self):
        self.assertEqual(normalize("  Ｍａｉｃａ！", "en_maica"), "maica")
        self.assertEqual(normalize("マイカ", "ja_maica"), "まいか")
        self.assertEqual(normalize("Ｍａｉｃａ", "vi_maica"), "maica")

    def test_alias_overrides_are_explicit(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"vi_maica": ["maika"]}', encoding="utf-8")
            changed = load_aliases(str(path), self.aliases)
            self.assertTrue(match_wake_phrase("maika", "vi_maica", changed))
            self.assertFalse(match_wake_phrase("maica", "vi_maica", changed))
            self.assertEqual(changed["ja_maica"], self.aliases["ja_maica"])

    def test_rejects_invalid_alias_structure(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "aliases.json"
            path.write_text('{"unknown": ["foo"]}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_aliases(str(path), self.aliases)


class ConfigurationTests(unittest.TestCase):
    def test_valid_default_config(self):
        cfg = load_config(DEFAULT_CONFIG_PATH)
        self.assertIn("keywords", cfg)
        for kw_id in ("vi_maica", "en_maica", "ja_maica"):
            self.assertIn(kw_id, cfg["keywords"])
            self.assertTrue(cfg["keywords"][kw_id]["enabled"])

    def test_rejects_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            load_config(Path("/non/existent/path/keywords.json"))

    def test_rejects_malformed_json(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json") as f:
            f.write("{invalid_json:")
            f.flush()
            with self.assertRaises(ValueError):
                load_config(Path(f.name))

    def test_rejects_invalid_schema(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json") as f:
            f.write(json.dumps({"keywords": {"test": {"language": "vi"}}}))
            f.flush()
            with self.assertRaises(ValueError):
                load_config(Path(f.name))

    def test_disabled_keyword_filtering(self):
        cfg = {
            "keywords": {
                "kw1": {"language": "vi", "display_name": "K1", "model": "m1", "aliases": ["a"], "enabled": False},
                "kw2": {"language": "en", "display_name": "K2", "model": "m2", "aliases": ["b"], "enabled": True},
            }
        }
        active = get_active_keywords(cfg)
        self.assertNotIn("kw1", active)
        self.assertIn("kw2", active)


class DummyRecognizer:
    def __init__(self, model, rate):
        self.model = model
        self.calls = 0

    def AcceptWaveform(self, pcm):
        self.calls += 1
        return self.calls == 2

    def Result(self):
        return json.dumps({"text": "maica" if self.model == "en" else ""})

    def FinalResult(self):
        return json.dumps({"text": "please" if self.model == "en" else ""})


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
                wav, DummyVosk, {"en_maica": "en", "vi_maica": "vi"}
            )
            self.assertEqual(transcripts["en_maica"], "maica please")
            self.assertEqual(transcripts["vi_maica"], "")
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
