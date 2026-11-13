"""Dependency-light tests for manifest validation, audio windows and event logic."""
from __future__ import annotations

import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location("souta_kws", Path(__file__).parent / "souta_kws.py")
kws = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kws)


class PipelineTests(unittest.TestCase):
    def test_audio_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "voice.wav"
            audio = np.arange(16000, dtype=np.int16)
            kws.write_wav(path, audio)
            np.testing.assert_array_equal(kws.read_wav(path), audio)

    def test_short_and_bad_audio(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "short.wav"
            kws.write_wav(path, np.ones(100, dtype=np.int16))
            with self.assertRaises(ValueError):
                kws.read_wav(path)

    def test_center_window(self):
        rng = np.random.default_rng(42)
        x = np.full(8000, 300, dtype=np.int16)
        y = kws.window_audio(x, rng)
        self.assertEqual(len(y), kws.WINDOW)
        self.assertEqual(int((y != 0).sum()), len(x))

    def test_augmentation_dtype_and_bounds(self):
        rng = np.random.default_rng(42)
        x = np.full(kws.WINDOW, 1000, dtype=np.int16)
        y = kws.augment_pcm(x, rng, [])
        self.assertEqual(y.dtype, np.dtype("int16"))
        self.assertEqual(len(y), kws.WINDOW)

    def test_group_assignment_keeps_speakers_together(self):
        groups = {f"speaker:{i}": {"positive", "negative"} for i in range(20)}
        groups.update({f"session:{i}": {"ambient"} for i in range(6)})
        assignments = kws.group_assignment(groups, seed=42)
        for category in ("positive", "negative", "ambient"):
            covered = {assignments[g] for g, cats in groups.items() if category in cats}
            self.assertEqual(covered, set(kws.SPLITS))

    def test_reject_leaky_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            d = Path(directory)
            audio = d / "a.wav"
            kws.write_wav(audio, np.ones(16000, dtype=np.int16))
            with (d / "manifest.csv").open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["path", "label", "group", "split"])
                w.writeheader()
                w.writerow({"path": "a.wav", "label": "positive",
                            "group": "speaker:01", "split": "train"})
                w.writerow({"path": "a.wav", "label": "negative",
                            "group": "speaker:01", "split": "test"})
            with self.assertRaisesRegex(ValueError, "leakage"):
                kws.load_manifest(d / "manifest.csv")

    def test_trigger_cooldown_and_rearm(self):
        scores = [0.1] * 25 + [0.9] * 10 + [0.1] * 30 + [0.9] * 10
        self.assertEqual(kws.count_triggers(scores, 0.5, cooldown_s=1), 2)
        self.assertEqual(kws.count_triggers(scores, 0.95), 0)


if __name__ == "__main__":
    unittest.main()
