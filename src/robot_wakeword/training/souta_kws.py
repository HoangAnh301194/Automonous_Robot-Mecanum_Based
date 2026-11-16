#!/usr/bin/env python3
"""Souta KWS training CLI; separate from ROS runtime and the legacy Vosk probe.

Commands: record, prepare, extract, train, calibrate, evaluate.
All existing command names and arguments remain supported.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Public re-exports preserve imports in existing test_pipeline.py and scripts.
from kws_core.constants import SAMPLE_RATE, WINDOW, HOP, CATEGORIES, SPLITS
from kws_core.audio import read_wav, write_wav, record, window_audio, augment_pcm
from kws_core.dataset import group_assignment, prepare, load_manifest
from kws_core.features import get_feature_extractor, extract
from kws_core.trainer import make_head, train
from kws_core.evaluation import (
    stream_scores, count_triggers, assess_recordings,
    get_stream_model, calibrate, evaluate,
)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    s = sub.add_parser("record", help="Record PCM WAV from PulseAudio on Linux")
    s.add_argument("--output", required=True, type=Path)
    s.add_argument("--duration", type=float, default=3)
    s.add_argument("--source", default="@DEFAULT_SOURCE@")
    s.set_defaults(func=record)
    s = sub.add_parser("prepare", help="Validate WAV data and speaker-disjoint splits")
    s.add_argument("--raw", type=Path, default=Path("data/raw"))
    s.add_argument("--manifest", type=Path, default=Path("data/manifest.csv"))
    s.add_argument("--seed", type=int, default=42)
    s.set_defaults(func=prepare)
    s = sub.add_parser("extract", help="Embed WAVs with frozen openWakeWord front-end")
    s.add_argument("--manifest", type=Path, default=Path("data/manifest.csv"))
    s.add_argument("--output", type=Path, default=Path("runs/features"))
    s.add_argument("--batch", type=int, default=16)
    s.add_argument("--augment", type=int, default=4)
    s.add_argument("--seed", type=int, default=42)
    s.set_defaults(func=extract)
    s = sub.add_parser("train", help="Train MLP head and export ONNX")
    s.add_argument("--features", type=Path, default=Path("runs/features"))
    s.add_argument("--output", type=Path, default=Path("runs/souta"))
    s.add_argument("--epochs", type=int, default=80)
    s.add_argument("--patience", type=int, default=10)
    s.add_argument("--batch", type=int, default=64)
    s.add_argument("--lr", type=float, default=0.001)
    s.add_argument("--seed", type=int, default=42)
    s.add_argument("--cpu", action="store_true")
    s.set_defaults(func=train)
    s = sub.add_parser("calibrate", help="Tune threshold with validation streaming audio")
    s.add_argument("--manifest", type=Path, default=Path("data/manifest.csv"))
    s.add_argument("--model", type=Path, default=Path("runs/souta/souta.onnx"))
    s.add_argument("--output", type=Path, default=Path("runs/souta/calibration.json"))
    s.add_argument("--target-far", type=float, default=0.5)
    s.add_argument("--min-negative-hours", type=float, default=1)
    s.set_defaults(func=calibrate)
    s = sub.add_parser("evaluate", help="Independent test recall and false accepts/hour")
    s.add_argument("--manifest", type=Path, default=Path("data/manifest.csv"))
    s.add_argument("--model", type=Path, default=Path("runs/souta/souta.onnx"))
    s.add_argument("--calibration", type=Path, default=Path("runs/souta/calibration.json"))
    s.add_argument("--output", type=Path, default=Path("runs/souta/test_report.json"))
    s.add_argument("--min-negative-hours", type=float, default=1)
    s.set_defaults(func=evaluate)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args) or 0
    except (ValueError, OSError, ImportError, RuntimeError) as exc:
        print(f"[souta-kws] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
