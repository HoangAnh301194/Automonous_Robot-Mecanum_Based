"""Frozen openWakeWord embedding extraction with training-only augmentation."""
from __future__ import annotations

from pathlib import Path

from .audio import augment_pcm, read_wav, window_audio
from .constants import WINDOW, SPLITS
from .dataset import load_manifest


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
                if category == "positive":
                    repeats = args.augment
                else:
                    # Sample a wider variety of locations from long negative streams.
                    repeats = min(100, max(2, args.augment, len(audio) // WINDOW))
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
