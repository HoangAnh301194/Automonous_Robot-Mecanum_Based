"""Smoke test to verify that actual pretrained Vosk models load into memory."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

from robot_wakeword.probe import DEFAULT_CONFIG_PATH, get_active_keywords, load_config, load_models


class PretrainedModelSmokeTest(unittest.TestCase):
    def test_load_all_pretrained_models(self):
        models_dir = Path.home() / "vosk-models"
        if not models_dir.is_dir():
            self.skipTest(f"Models directory not found at {models_dir}. Run download_models.py first.")

        try:
            import vosk  # noqa: F401
        except ImportError:
            self.skipTest("vosk is not installed in the current environment.")

        config = load_config(DEFAULT_CONFIG_PATH)
        active = get_active_keywords(config)
        vosk_module, models = load_models(models_dir, active)

        self.assertEqual(len(models), len(active))
        for kw_id in active:
            self.assertIn(kw_id, models)
            self.assertIsNotNone(models[kw_id])
        print(f"\n[SmokeTest] Successfully loaded {len(models)} Vosk models from {models_dir}")


if __name__ == "__main__":
    unittest.main()
