"""Train a small PyTorch classifier head and export its ONNX graph."""
from __future__ import annotations

import json
from pathlib import Path

from .constants import SAMPLE_RATE, HOP


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
            frozen = {name: tensor.detach().cpu().clone()
                      for name, tensor in model.state_dict().items()}
            torch.save({"state_dict": frozen, "epoch": epoch,
                        "input_shape": [16, 96]}, destination / "best.pt")
        else:
            patience += 1
            if patience >= args.patience:
                break
    model = make_head()
    model.load_state_dict(torch.load(destination / "best.pt", map_location="cpu",
                                     weights_only=True)["state_dict"])
    model.eval()
    export = destination / "souta.onnx"
    torch.onnx.export(model, torch.zeros(1, 16, 96), str(export),
                      opset_version=13, dynamo=False,
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
