"""Streaming evaluation, hysteresis/refractory gating and validation calibration."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from .audio import read_wav
from .constants import SAMPLE_RATE, HOP
from .dataset import load_manifest


def stream_scores(audio, model):
    """Score one recording through openWakeWord's real streaming pipeline."""
    import numpy as np
    model.reset()
    # For short utterances, surround speech with silence to fill embedding history.
    if len(audio) < 3 * SAMPLE_RATE:
        audio = np.pad(audio, (SAMPLE_RATE, SAMPLE_RATE))
    scores = []
    for start in range(0, len(audio), HOP):
        frame = audio[start:start + HOP]
        if len(frame) < HOP:
            frame = np.pad(frame, (0, HOP - len(frame)))
        pred = model.predict(frame.astype(np.int16))
        if pred:
            scores.append(float(next(iter(pred.values()))))
    return scores

def count_triggers(scores, threshold, cooldown_s=2.0):
    """Count threshold-crossing events, with hysteresis and a refractory interval."""
    events, next_allowed, armed = 0, 0, True
    for frame, score in enumerate(scores):
        time_s = frame * HOP / SAMPLE_RATE
        if score < threshold * 0.55:
            armed = True
        elif score >= threshold and armed and time_s >= next_allowed:
            events += 1
            armed = False
            next_allowed = time_s + cooldown_s
    return events

def assess_recordings(rows, model, threshold):
    """Record-level recall and false wake-ups per measured negative audio hour."""
    true_count, hits, false_events, neg_seconds = 0, 0, 0, 0.0
    detail = []
    for row in rows:
        audio = read_wav(row["path"])
        scores = stream_scores(audio, model)
        count = count_triggers(scores, threshold)
        is_pos = row["label"] == "positive"
        if is_pos:
            true_count += 1
            hits += int(count > 0)
        else:
            false_events += count
            neg_seconds += len(audio) / SAMPLE_RATE
        detail.append({"path": row["path"], "label": row["label"],
                       "max_score": max(scores, default=0), "triggers": count})
    hours = neg_seconds / 3600
    return {"recall": hits / true_count if true_count else None,
            "detected": hits, "positive_total": true_count,
            "false_accepts": false_events, "negative_hours": round(hours, 4),
            "false_accepts_per_hour": false_events / hours if hours > 0 else None,
            "samples": detail}

def get_stream_model(path):
    from openwakeword.model import Model
    return Model(wakeword_models=[str(Path(path).resolve())],
                 inference_framework="onnx", enable_speex_noise_suppression=False)

def calibrate(args):
    """Tune event threshold on VAL only; never inspect TEST for calibration."""
    rows = [r for r in load_manifest(args.manifest) if r["split"] == "val"]
    model = get_stream_model(args.model)
    # Compute streaming scores once, then replay all thresholds without more inference.
    trials = []
    for row in rows:
        audio = read_wav(row["path"])
        trials.append((row["label"], stream_scores(audio, model), len(audio) / SAMPLE_RATE))
    neg_hours = sum(secs for label, scores, secs in trials if label != "positive") / 3600
    if neg_hours <= 0:
        raise ValueError("Need held-out negative recordings to calibrate false alarms")
    candidates = []
    for integer in range(5, 100, 5):
        th = integer / 100
        tp = sum(count_triggers(scores, th) > 0 for label, scores, secs in trials
                 if label == "positive")
        pos_n = sum(label == "positive" for label, _, _ in trials)
        fa = sum(count_triggers(scores, th) for label, scores, _ in trials
                 if label != "positive")
        candidates.append({"threshold": th, "recall": tp / pos_n if pos_n else 0.0,
                           "false_accepts": fa, "false_accepts_per_hour": fa / neg_hours})
    feasible = [c for c in candidates if c["false_accepts_per_hour"] <= args.target_far]
    if not feasible:
        raise ValueError("No threshold meets target FAR on validation data; collect more "
                         "hard negatives/retrain instead of silently passing")
    chosen = max(feasible, key=lambda c: (c["recall"], -c["threshold"]))
    report = {"chosen": chosen, "candidates": candidates, "negative_hours": neg_hours,
              "target_far": args.target_far, "min_negative_hours": args.min_negative_hours,
              "data_sufficient": neg_hours >= args.min_negative_hours}
    if not report["data_sufficient"]:
        print(f"WARNING: only {neg_hours:.2f} negative hours (target >= "
              f"{args.min_negative_hours:.1f}); threshold is PRELIMINARY", file=sys.stderr)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"calibration": str(out), **report}, indent=2))

def evaluate(args):
    """Evaluate fixed calibrated threshold on speaker/session-disjoint TEST only."""
    calibration = json.loads(Path(args.calibration).read_text(encoding="utf-8"))
    threshold = float(calibration["chosen"]["threshold"])
    model = get_stream_model(args.model)
    rows = [r for r in load_manifest(args.manifest) if r["split"] == "test"]
    report = {"model": str(args.model), "threshold": threshold, "split": "test",
              **assess_recordings(rows, model, threshold)}
    if report["negative_hours"] < args.min_negative_hours:
        report["warning"] = "Insufficient negative audio duration for strong FAR claim"
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
    return 0
