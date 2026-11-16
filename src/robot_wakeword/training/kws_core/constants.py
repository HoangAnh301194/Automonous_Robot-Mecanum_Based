"""Shared audio and dataset conventions for Souta KWS."""

SAMPLE_RATE = 16000
WINDOW = 32000  # Two seconds at 16 kHz
HOP = 1280  # openWakeWord: 80 ms of audio at 16 kHz
CATEGORIES = {"positive", "negative", "hard_negative", "ambient"}
SPLITS = ("train", "val", "test")
