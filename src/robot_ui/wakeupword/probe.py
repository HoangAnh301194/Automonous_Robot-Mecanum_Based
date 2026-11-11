#!/usr/bin/env python3
"""Phase 1 offline wake-phrase baseline: Vosk ASR keyword matching.

Self-contained module for Souta robot. Independent of ROS 2 and Kiosk UI.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import resource
import subprocess
import sys
import time
import unicodedata
import wave
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).parent / "config" / "keywords.json"

# Fallback definitions for backward-compatibility with tests & direct imports
MODEL_DIRS = {
    "vi_maica": "vosk-model-small-vn-0.4",
    "en_maica": "vosk-model-small-en-us-0.15",
    "ja_maica": "vosk-model-small-ja-0.22",
}

ALIASES = {
    "vi_maica": ["maica"],
    "en_maica": ["maica"],
    "ja_maica": ["まいか", "マイカ"],
}


def load_config(config_path: Path | None = None) -> dict:
    """Load keyword and model configurations from JSON."""
    path = config_path or DEFAULT_CONFIG_PATH
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in config {path}: {exc}") from exc

    if "keywords" not in data or not isinstance(data["keywords"], dict):
        raise ValueError("Config must have a top-level 'keywords' object")

    for kw_id, kw_cfg in data["keywords"].items():
        for field in ("language", "display_name", "model", "aliases", "enabled"):
            if field not in kw_cfg:
                raise ValueError(f"Keyword '{kw_id}' missing required field: {field}")
        if not isinstance(kw_cfg["aliases"], list) or not kw_cfg["aliases"]:
            raise ValueError(f"Keyword '{kw_id}' aliases must be a non-empty list")

    return data


def get_active_keywords(config: dict) -> dict:
    """Return dictionary of enabled keywords."""
    return {k: v for k, v in config.get("keywords", {}).items() if v.get("enabled", True)}


def normalize(text: str, language_or_label: str) -> str:
    """Normalize text with Unicode NFKC, casefolding, and Japanese kana conversion."""
    text = unicodedata.normalize("NFKC", text).casefold()
    if language_or_label == "ja" or language_or_label.startswith("ja_"):
        # Convert Katakana to Hiragana (U+30A1 - U+30F6 -> U+3041 - U+3096)
        text = "".join(
            chr(ord(c) - 0x60) if 0x30A1 <= ord(c) <= 0x30F6 else c
            for c in text
        )
        return "".join(c for c in text if c.isalnum())
    return " ".join(re.findall(r"[^\W_]+", text, re.UNICODE))


def match_wake_phrase(text: str, label: str, aliases: dict | None = None) -> bool:
    """Check if recognized text matches any configured alias for the keyword."""
    variants = (aliases or ALIASES).get(label, [])
    normalized = normalize(text, label)
    for variant in variants:
        phrase = normalize(variant, label)
        if not phrase:
            continue
        if label.startswith("ja_") or label == "ja":
            if phrase in normalized:
                return True
        elif re.search(r"(?<!\w)" + re.escape(phrase) + r"(?!\w)", normalized):
            return True
    return False


def load_aliases(path: str | None, base_aliases: dict | None = None) -> dict:
    """Load and validate alias overrides from a JSON file."""
    aliases = {name: list(values) for name, values in (base_aliases or ALIASES).items()}
    if path:
        loaded = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError("Aliases must be a JSON object keyed by wake-word ID")
        for label, variants in loaded.items():
            if label not in aliases and label not in MODEL_DIRS:
                raise ValueError(f"Invalid alias entry: {label}")
            if not isinstance(variants, list) or not all(isinstance(v, str) and v.strip() for v in variants):
                raise ValueError(f"Empty or non-string alias for {label}")
            aliases[label] = variants
    return aliases


def load_models(models_dir: Path, active_keywords: dict | None = None):
    """Load Vosk library and active language models."""
    try:
        import vosk
    except ImportError as exc:
        raise SystemExit(
            "Missing vosk library. Please run:\n"
            "  bash install.sh\n"
            "or: pip install -r requirements.txt"
        ) from exc

    vosk.SetLogLevel(-1)
    models = {}
    items = active_keywords if active_keywords is not None else {
        k: {"model": m} for k, m in MODEL_DIRS.items()
    }
    for kw_id, kw_cfg in items.items():
        model_name = kw_cfg.get("model", MODEL_DIRS.get(kw_id))
        folder = models_dir / model_name
        if not folder.is_dir():
            raise SystemExit(
                f"Missing model folder for {kw_id}: {folder}\n"
                f"Please download it via: python download_models.py --models-dir {models_dir}"
            )
        models[kw_id] = vosk.Model(str(folder))
    return vosk, models


def open_wav(path: Path):
    """Open and validate 16-bit mono 16kHz PCM WAV."""
    handle = wave.open(str(path), "rb")
    if (handle.getnchannels(), handle.getsampwidth(), handle.getframerate()) != (1, 2, 16000):
        handle.close()
        raise ValueError(f"Expected mono 16-bit PCM 16000 Hz WAV: {path}")
    return handle


def recognize_file(path: Path, vosk_module, models: dict) -> tuple[dict, float, float]:
    """Recognize a single WAV file across all models."""
    recognizers = {name: vosk_module.KaldiRecognizer(model, 16000) for name, model in models.items()}
    start = time.monotonic()
    segments = {name: [] for name in models}
    with open_wav(path) as wav:
        duration = wav.getnframes() / 16000
        while True:
            pcm = wav.readframes(1600)
            if not pcm:
                break
            for name, recognizer in recognizers.items():
                if recognizer.AcceptWaveform(pcm):
                    segment = json.loads(recognizer.Result()).get("text", "")
                    if segment:
                        segments[name].append(segment)
    transcripts = {}
    for name, recognizer in recognizers.items():
        final = json.loads(recognizer.FinalResult()).get("text", "")
        if final:
            segments[name].append(final)
        transcripts[name] = " ".join(segments[name])
    return transcripts, time.monotonic() - start, duration


def evaluate(manifest: Path, vosk_module, models: dict, aliases: dict, report: Path | None):
    """Evaluate offline manifest for recall and false accept rate."""
    records = []
    with manifest.open(newline="", encoding="utf-8-sig") as fp:
        for row in csv.DictReader(fp):
            if "file" not in row or "expected" not in row:
                raise ValueError("Manifest needs columns: file,expected")
            path = Path(row["file"])
            if not path.is_absolute():
                path = manifest.parent / path
            expected = row["expected"].strip() or "none"
            if expected != "none" and expected not in models:
                raise ValueError(f"Unknown expected label: {expected}")
            transcripts, elapsed, duration = recognize_file(path, vosk_module, models)
            detected = [label for label, transcript in transcripts.items()
                        if match_wake_phrase(transcript, label, aliases)]
            outcome = ("correct" if expected in detected and len(detected) == 1 else
                       "correct_reject" if expected == "none" and not detected else
                       "false_accept" if expected == "none" else
                       "miss" if not detected else "confusion")
            record = dict(file=str(path), expected=expected, detected=detected,
                          outcome=outcome, decode_seconds=round(elapsed, 3),
                          audio_seconds=round(duration, 3), transcripts=transcripts)
            records.append(record)
            print(f"{outcome:14s} expected={expected:15s} detected={','.join(detected) or '-':18s} "
                  f"RTF={elapsed / duration if duration else 0:.2f} {path.name}")

    positives = [r for r in records if r["expected"] != "none"]
    negatives = [r for r in records if r["expected"] == "none"]
    positive_correct = sum(r["outcome"] == "correct" for r in positives)
    false_accepts = sum(bool(r["detected"]) for r in negatives)
    negative_hours = sum(r["audio_seconds"] for r in negatives) / 3600
    by_keyword = {
        name: {
            "total": sum(r["expected"] == name for r in positives),
            "correct": sum(r["expected"] == name and r["outcome"] == "correct" for r in positives),
        }
        for name in models
    }
    summary = {
        "positive_recall": positive_correct / len(positives) if positives else None,
        "positive_count": len(positives),
        "negative_count": len(negatives),
        "false_accepts": false_accepts,
        "negative_audio_hours": round(negative_hours, 4),
        "false_accepts_per_hour": false_accepts / negative_hours if negative_hours else None,
        "by_keyword": by_keyword,
        "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }
    print("\nSUMMARY\n" + json.dumps(summary, ensure_ascii=False, indent=2))
    if report:
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps({"summary": summary, "samples": records},
                                     indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Wrote report: {report}")


def capture_cmd(source: str) -> list[str]:
    return ["parec", "-d", source, "--rate=16000", "--channels=1",
            "--format=s16le", "--latency-msec=50", "--raw"]


def record_sample(path: Path, duration: float, source: str):
    """Record a PCM sample directly to 16kHz mono WAV."""
    path.parent.mkdir(parents=True, exist_ok=True)
    command = ["timeout", "--signal=TERM", "--kill-after=1s",
               f"{duration}s", *capture_cmd(source)]
    result = subprocess.run(command, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, timeout=duration + 5,
                            check=False)
    pcm = result.stdout[:round(duration * 16000) * 2]
    if len(pcm) < 3200:
        raise RuntimeError(f"No usable PCM received from microphone (exit={result.returncode})")
    with wave.open(str(path), "wb") as fp:
        fp.setnchannels(1)
        fp.setsampwidth(2)
        fp.setframerate(16000)
        fp.writeframes(pcm)
    print(f"Recorded {len(pcm) / 32000:.2f}s to {path}")


def live_probe(vosk_module, models: dict, aliases: dict, source: str, cooldown: float,
               debug_transcripts: bool = False, debug_partials: bool = False):
    """One parec capture feeding all recognizers concurrently."""
    recognizers = {name: vosk_module.KaldiRecognizer(model, 16000) for name, model in models.items()}
    process = subprocess.Popen(capture_cmd(source), stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, bufsize=0)
    next_allowed = 0.0
    audio_seconds = 0.0
    cpu_seconds = 0.0
    last_stat = time.monotonic()
    last_partial_log = {name: 0.0 for name in models}
    last_partial_text = {name: "" for name in models}
    print("Listening; detection is based on final ASR segments. Ctrl+C to exit.", flush=True)
    if debug_transcripts or debug_partials:
        print("DEBUG: ASR transcripts may contain private speech. Do not share raw logs without consent.",
              file=sys.stderr, flush=True)
    try:
        while True:
            pcm = process.stdout.read(3200)
            if not pcm:
                if process.poll() is not None:
                    raise RuntimeError(f"parec exited with code {process.returncode}")
                continue
            audio_seconds += len(pcm) / 32000
            t0 = time.monotonic()
            for name, recognizer in recognizers.items():
                if recognizer.AcceptWaveform(pcm):
                    transcript = json.loads(recognizer.Result()).get("text", "")
                    if debug_transcripts and transcript.strip():
                        print(json.dumps({
                            "event": "asr_final", "model": name, "text": transcript
                        }, ensure_ascii=False), flush=True)
                    if time.monotonic() >= next_allowed and match_wake_phrase(transcript, name, aliases):
                        print(json.dumps({
                            "event": "wake_detected", "keyword": "maica", "detector": name,
                            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                        }), flush=True)
                        next_allowed = time.monotonic() + cooldown
                elif debug_partials:
                    now = time.monotonic()
                    if now - last_partial_log[name] >= 1.0:
                        partial = json.loads(recognizer.PartialResult()).get("partial", "").strip()
                        if partial and partial != last_partial_text[name]:
                            print(json.dumps({
                                "event": "asr_partial", "model": name, "text": partial
                            }, ensure_ascii=False), flush=True)
                            last_partial_text[name] = partial
                        last_partial_log[name] = now
            cpu_seconds += time.monotonic() - t0
            if time.monotonic() - last_stat > 10:
                print(f"[perf] decode wall/audio ratio={cpu_seconds / audio_seconds:.2f}; "
                      f"peak RSS={resource.getrusage(resource.RUSAGE_SELF).ru_maxrss} KiB",
                      file=sys.stderr, flush=True)
                last_stat = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        process.terminate()
        try:
            process.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("record", "evaluate", "live"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH,
                        help="Path to keywords.json configuration")
    parser.add_argument("--models-dir", type=Path, default=Path.home() / "vosk-models")
    parser.add_argument("--source", default="@DEFAULT_SOURCE@", help="PulseAudio source name")
    parser.add_argument("--duration", type=float, default=4, help="Record duration, seconds")
    parser.add_argument("--output", type=Path, help="WAV output path for record")
    parser.add_argument("--manifest", type=Path, help="CSV file for evaluate")
    parser.add_argument("--aliases", help="Optional JSON alias overrides")
    parser.add_argument("--report", type=Path, help="JSON output containing transcripts")
    parser.add_argument("--cooldown", type=float, default=2.0, help="Cooldown seconds in live mode")
    parser.add_argument("--debug-transcripts", action="store_true",
                        help="Print finalized Vosk transcripts (privacy-sensitive; live mode only)")
    parser.add_argument("--debug-partials", action="store_true",
                        help="Print throttled partial Vosk transcripts (privacy-sensitive; live mode only)")
    args = parser.parse_args(argv)

    if args.command == "record":
        if not args.output or not 0.5 <= args.duration <= 60:
            parser.error("record requires --output and 0.5 <= --duration <= 60")
        record_sample(args.output, args.duration, args.source)
        return 0

    if args.config.is_file():
        config = load_config(args.config)
        active = get_active_keywords(config)
        base_aliases = {k: v["aliases"] for k, v in active.items()}
    else:
        active = {k: {"model": m} for k, m in MODEL_DIRS.items()}
        base_aliases = ALIASES

    aliases = load_aliases(args.aliases, base_aliases)
    models_dir = args.models_dir.expanduser().resolve()
    vosk_module, models = load_models(models_dir, active)

    if args.command == "evaluate":
        if not args.manifest:
            parser.error("evaluate requires --manifest")
        evaluate(args.manifest, vosk_module, models, aliases, args.report)
    else:
        live_probe(vosk_module, models, aliases, args.source, max(0, args.cooldown),
                   debug_transcripts=args.debug_transcripts,
                   debug_partials=args.debug_partials)
    return 0


if __name__ == "__main__":
    sys.exit(main())
