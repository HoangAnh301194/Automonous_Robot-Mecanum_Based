"""Validate the corpus, split speaker/session groups, and maintain the manifest."""
from __future__ import annotations

import csv
import json
import random
from collections import defaultdict
from pathlib import Path

from .audio import read_wav
from .constants import CATEGORIES, SPLITS


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
