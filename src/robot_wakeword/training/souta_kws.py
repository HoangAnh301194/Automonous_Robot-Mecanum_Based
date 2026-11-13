#!/usr/bin/env python3
"""Souta KWS: recording -> speaker split -> embedding -> classifier -> ONNX -> FAR evaluation.

Training is intentionally independent from the ROS 2 Kiosk and the legacy Vosk probe.
Requires real 16 kHz, 16-bit mono WAV recordings. No fabricated training data.
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import subprocess
import sys
import wave
from collections import defaultdict
from pathlib import Path

SAMPLE_RATE = 16000
WINDOW = 32000
HOP = 1280  # 80 ms, openWakeWord's streaming increment
CATEGORIES = {"positive", "negative", "hard_negative", "ambient"}
SPLITS = ("train", "val", "test")


def read_wav(path):
    """Read PCM S16 mono 16 kHz; reject silent/empty files and unconverted input."""
    import numpy as np
    with wave.open(str(path), "rb") as wav:
        if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) != (1, 2, SAMPLE_RATE):
            raise ValueError(f"Expected 16 kHz mono S16 PCM WAV: {path}")
        raw = wav.readframes(wav.getnframes())
    audio = np.frombuffer(raw, dtype="<i2").copy()
    if len(audio) < SAMPLE_RATE // 4:
        raise ValueError(f"Audio shorter than 0.25 s: {path}")
    if not np.any(audio):
        raise ValueError(f"Silent/empty audio file: {path}")
    return audio


def write_wav(path, audio):
    import numpy as np
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(np.asarray(audio, dtype="<i2").tobytes())


def record(args):
    """Collect a sample without calling any speech recognizer."""
    duration = float(args.duration)
    if not 0.5 <= duration <= 120:
        raise ValueError("Recording duration must be 0.5–120 seconds")
    cmd = ["parec", "-d", args.source, "--rate=16000", "--channels=1",
           "--format=s16le", "--raw"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        raw = proc.stdout.read(round(duration * SAMPLE_RATE) * 2)
    finally:
        proc.terminate()
        try:
            proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
    if len(raw) < SAMPLE_RATE // 2:
        raise RuntimeError("Could not capture microphone audio; inspect pactl list short sources")
    import numpy as np
    write_wav(args.output, np.frombuffer(raw, dtype="<i2"))
    print(f"Saved: {args.output} ({len(raw) / 32000:.2f} seconds)")


def group_assignment(groups, seed=42):
    """Assign every speaker/session to one split, with mandatory coverage per class.

    A speaker appearing in positive and negative folders remains in the same split.
    """
    rng = random.Random(seed)
    assignment = {}
    def place(kind):
        candidates = [g for g, labels in groups.items() if kind in labels]
        rng.shuffle(candidates)
        if len(candidates) < 3:
            raise ValueError(f"Need >=3 independent {kind} groups for train/val/test; found {len(candidates)}")
        for split in SPLITS:
            if not any(split == s and kind in groups[g] for g, s in assignment.items()):
                available = [g for g in candidates if g not in assignment]
                if not available:
                    raise ValueError(f"Not enough groups to cover split {split} for {kind}")
                assignment[available[0]] = split
        for g in candidates:
            if g not in assignment:
                counts = {s: sum(v == s for v in assignment.values()) for s in SPLITS}
                total = max(1, sum(counts.values()))
                targets = {"train": 0.70, "val": 0.15, "test": 0.15}
                split = min(SPLITS, key=lambda s: (counts[s] + 1) / targets[s])
                assignment[g] = split
    for category in ("positive", "negative", "hard_negative", "ambient"):
        present = any(category in labels for labels in groups.values())
        if present:
            place(category)
    for g in groups:
        if g not in assignment:
            assignment[g] = "train"
    return assignment


def prepare(args):
    """Index data/raw/{category}/{speaker_or_session}/*.wav with disjoint speaker splits."""
    raw_dir = Path(args.raw)
    items = []
    groups = defaultdict(set)
    for category in sorted(CATEGORIES):
        folder = raw_dir / category
        for file in sorted(folder.glob("*/*.wav")):
            rel = file.relative_to(raw_dir)
            identity = rel.parts[1]
            if not identity:
                raise ValueError(f"Missing speaker/session identity for {file}")
            # ambient recordings are tagged by their capture session, not speaker.
            group = ("session:" if category == "ambient" else "speaker:") + identity
            read_wav(file)  # validate BEFORE manifest generation
            groups[group].add(category)
            items.append({"path": str(file.resolve()), "label": category, "group": group})
    if not items or "positive" not in {i["label"] for i in items}:
        raise ValueError("No positive files. Expected raw/positive/<speaker_id>/*.wav")
    if "negative" not in {i["label"] for i in items}:
        raise ValueError("No negatives. Expected raw/negative/<speaker_id>/*.wav")
    allocation = group_assignment(groups, seed=args.seed)
    for item in items:
        item["split"] = allocation[item["group"]]
    for category in ("positive", "negative"):
        for split in SPLITS:
            if not any(i["label"] == category and i["split"] == split for i in items):
                raise ValueError(f"No {category} samples in {split}. Add speakers and try another seed.")
    output = Path(args.manifest)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["path", "label", "group", "split"])
        writer.writeheader()
        writer.writerows(items)
    print(json.dumps({"manifest": str(output), "records": len(items),
                      "groups": len(groups),
                      "splits": {s: sum(i["split"] == s for i in items) for s in SPLITS}},
                     indent=2))


def load_manifest(path):
    manifest = Path(path)
    with manifest.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("Empty manifest")
    by_group = {}
    for row in rows:
        if set(("path", "label", "group", "split")) - set(row):
            raise ValueError("Manifest requires path,label,group,split")
        if row["split"] not in SPLITS or row["label"] not in CATEGORIES:
            raise ValueError(f"Invalid manifest row: {row}")
        if row["group"] in by_group and by_group[row["group"]] != row["split"]:
            raise ValueError(f"Speaker/session leakage across splits: {row['group']}")
        by_group[row["group"]] = row["split"]
        p = Path(row["path"])
        if not p.is_absolute():
            p = manifest.parent / p
        if not p.is_file():
            raise FileNotFoundError(p)
        row["path"] = str(p.resolve())
    return rows


def window_audio(audio, rng, shift=False, random_crop=False):
    """Make a 2 s window centered on an utterance; random crop for negative recordings."""
    import numpy as np
    n = len(audio)
    if n >= WINDOW:
        if random_crop:
            start = int(rng.integers(0, n - WINDOW + 1))
        else:
            start = (n - WINDOW) // 2
            if shift:
                start += int(rng.integers(-2000, 2001))
                start = max(0, min(n - WINDOW, start))
        return audio[start:start + WINDOW].copy()
    out = np.zeros(WINDOW, dtype=np.int16)
    start = (WINDOW - n) // 2
    if shift:
        start += int(rng.integers(-2000, 2001))
        start = max(0, min(WINDOW - n, start))
    out[start:start + n] = audio
    return out


def augment_pcm(audio, rng, noise_pool):
    """Apply mild gain and optionally additive room noise at a sampled SNR."""
    import numpy as np
    x = audio.astype(np.float32) / 32768
    gain = float(rng.uniform(0.65, 1.4))
    x *= gain
    if noise_pool and rng.random() < 0.75:
        noise = window_audio(noise_pool[int(rng.integers(0, len(noise_pool)))],
                             rng, random_crop=True).astype(np.float32) / 32768
        p_signal = float(np.mean(x * x))
        p_noise = float(np.mean(noise * noise))
        if p_signal > 1e-7 and p_noise > 1e-9:
            snr_db = float(rng.uniform(5, 25))
            noise *= (p_signal / (p_noise * (10 ** (snr_db / 10)))) ** 0.5
            x += noise
    return np.clip(x * 32768, -32768, 32767).astype(np.int16)


def get_feature_extractor():
    """Official openWakeWord frozen mel + speech-embedding models (ONNX CPU)."""
    import openwakeword
    from openwakeword.utils import AudioFeatures
    # Download only shared feature extractors, not an unrelated wake-word classifier.
    openwakeword.utils.download_models(model_names=["__souta_custom_only__"])
    return AudioFeatures(device="cpu", inference_framework="onnx", ncpu=1)


def extract(args):
    """Extract openWakeWord 16x96 features, with augmentation ONLY on train split."""
    import numpy as np
    rows = load_manifest(args.manifest)
    rng = np.random.default_rng(args.seed)
    extractor = get_feature_extractor()
    dest = Path(args.output)
    dest.mkdir(parents=True, exist_ok=True)
    noises = [read_wav(row["path"]) for row in rows
              if row["label"] == "ambient" and row["split"] == "train"]
    for split in SPLITS:
        samples, labels, source_groups = [], [], []
        subset = [r for r in rows if r["split"] == split]
        for row in subset:
            audio = read_wav(row["path"])
            category = row["label"]
            # Long negative and ambient recordings contribute many windows.
            if split == "train":
                repeats = (args.augment if category == "positive" else
                           max(2, args.augment // 2) if category == "hard_negative" else
                           max(2, args.augment))
            else:
                repeats = 1 if category == "positive" else max(1, min(10, len(audio) // WINDOW))
            for i in range(repeats):
                pcm = window_audio(audio, rng, shift=split == "train" and category == "positive",
                                   random_crop=category != "positive")
                if split == "train" and i > 0:
                    pcm = augment_pcm(pcm, rng, noises)
                samples.append(pcm)
                labels.append(1 if category == "positive" else 0)
                source_groups.append(row["group"])
        if not samples or len(set(labels)) < 2:
            raise ValueError(f"Split {split} must contain both positive and negative examples")
        # Compute in bounded batches instead of keeping all embeddings in RAM during extraction.
        vectors = []
        for start in range(0, len(samples), args.batch):
            batch = np.stack(samples[start:start + args.batch])
            emb = np.asarray(extractor.embed_clips(batch, batch_size=len(batch), ncpu=1))
            if emb.shape[1:] != (16, 96):
                raise RuntimeError(f"Expected openWakeWord features (N,16,96), received {emb.shape}")
            vectors.append(emb.astype(np.float32))
        np.savez_compressed(dest / f"{split}.npz", x=np.concatenate(vectors),
                            y=np.asarray(labels, dtype=np.float32),
                            groups=np.asarray(source_groups))
        print(f"{split}: {len(labels)} examples; positive={sum(labels)}; saved {dest / (split + '.npz')}")


def make_head():
    import torch.nn as nn
    class SoutaHead(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(
                nn.Flatten(), nn.Linear(16 * 96, 128), nn.ReLU(),
                nn.Dropout(0.2), nn.Linear(128, 32), nn.ReLU(),
                nn.Linear(32, 1)
            )
        def logits(self, features):
            return self.net(features)
        def forward(self, features):
            return self.logits(features).sigmoid()
    return SoutaHead()


def train(args):
    """Train only the small head; choose checkpoint using validation loss."""
    import numpy as np
    import torch
    import torch.nn.functional as F
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    data = {}
    for split in ("train", "val"):
        path = Path(args.features) / f"{split}.npz"
        with np.load(path) as z:
            data[split] = (torch.from_numpy(z["x"]).float(), torch.from_numpy(z["y"]).float())
    xt, yt = data["train"]
    xv, yv = data["val"]
    if not (yt.sum() > 0 and (yt == 0).sum() > 0):
        raise ValueError("Both classes required in training split")
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    model = make_head().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    pos_weight = torch.tensor([((yt == 0).sum() / yt.sum()).item()], device=device)
    best_loss, best_epoch, patience = float("inf"), 0, 0
    destination = Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    for epoch in range(1, args.epochs + 1):
        model.train()
        perm = torch.randperm(len(xt))
        losses = []
        for indices in perm.split(args.batch):
            xx, yy = xt[indices].to(device), yt[indices].to(device)
            optimizer.zero_grad()
            loss = F.binary_cross_entropy_with_logits(
                model.logits(xx).flatten(), yy, pos_weight=pos_weight)
            loss.backward()
            optimizer.step()
            losses.append(float(loss.item()))
        model.eval()
        with torch.no_grad():
            probs = model(xv.to(device)).flatten()
            val_loss = float(F.binary_cross_entropy(probs, yv.to(device)).item())
            preds = probs >= 0.5
            recall = float(((preds & (yv.to(device) == 1)).sum() /
                           (yv.to(device) == 1).sum().clamp_min(1)).item())
        print(f"epoch={epoch:03d} train_loss={sum(losses)/len(losses):.4f}"
              f" val_loss={val_loss:.4f} val_recall@0.5={recall:.3f}")
        if val_loss < best_loss - 1e-4:
            best_loss, best_epoch, patience = val_loss, epoch, 0
            torch.save({"state_dict": model.cpu().state_dict(), "epoch": epoch,
                        "input_shape": [16, 96]}, destination / "best.pt")
            model.to(device)
        else:
            patience += 1
            if patience >= args.patience:
                break
    model = make_head()
    model.load_state_dict(torch.load(destination / "best.pt", map_location="cpu",
                                     weights_only=True)["state_dict"])
    model.eval()
    export = destination / "souta.onnx"
    torch.onnx.export(model, torch.zeros(1, 16, 96), str(export), opset_version=13,
                      input_names=["features"], output_names=["probability"],
                      dynamic_axes={"features": {0: "batch"}, "probability": {0: "batch"}})
    import onnx
    onnx.checker.check_model(str(export))
    metadata = {"model_name": "souta", "wake_phrase": "Sô ta / Sô ta ơi",
                "architecture": "openWakeWord frozen embedding + MLP head",
                "input_shape": [16, 96], "sample_rate": SAMPLE_RATE,
                "hop_samples": HOP, "best_epoch": best_epoch,
                "validation_loss": best_loss, "threshold": None}
    (destination / "model_metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False),
                                                     encoding="utf-8")
    print(f"Exported {export} (classifier only; also requires openWakeWord shared extractors)")


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
