"""File-only recorder storage. This module works without a microphone or GUI."""
from __future__ import annotations
import json
import os
import tempfile
import wave
from pathlib import Path

SAMPLE_RATE = 16000
DATA = Path(__file__).resolve().parent / "data"
LABELS = ("positive", "negative", "hard_negative", "ambient")

class RecorderStore:
    """Local-only speaker registry and non-overwriting WAV storage."""

    def __init__(self, data_dir=DATA):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.registry_file = self.data_dir / "recorder_registry.json"
        if self.registry_file.exists():
            self.registry = json.loads(self.registry_file.read_text(encoding="utf-8"))
            if not isinstance(self.registry, dict) or not all(
                isinstance(self.registry.get(k), list) for k in ("speakers", "sessions")
            ):
                raise ValueError("Registry không hợp lệ")
        else:
            self.registry = {"speakers": [], "sessions": []}

    def members(self, kind):
        return list(self.registry[kind])

    def register(self, kind, name):
        if kind not in ("speakers", "sessions"):
            raise ValueError("Loại đăng ký không hợp lệ")
        name = " ".join(name.strip().split())
        if not name or len(name) > 80 or any(ord(ch) < 32 for ch in name):
            raise ValueError("Nhập tên từ 1 đến 80 ký tự hợp lệ")
        if any(item["name"].casefold() == name.casefold() for item in self.registry[kind]):
            raise ValueError("Tên đã tồn tại, hãy chọn trong danh sách")
        prefix = "speaker" if kind == "speakers" else "session"
        number = 1
        ids = {x["id"] for x in self.registry[kind]}
        while f"{prefix}_{number:03d}" in ids:
            number += 1
        member = {"id": f"{prefix}_{number:03d}", "name": name}
        self.registry[kind].append(member)
        fd, tmp = tempfile.mkstemp(prefix=".registry_", dir=self.data_dir)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.registry, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(tmp, self.registry_file)
        finally:
            Path(tmp).unlink(missing_ok=True)
        return member

    def save_pcm(self, label, group_id, pcm):
        if label not in LABELS:
            raise ValueError("Nhãn không hợp lệ")
        kind = "sessions" if label == "ambient" else "speakers"
        if group_id not in {m["id"] for m in self.registry[kind]}:
            raise ValueError("Không tìm thấy người nói hoặc phiên môi trường")
        if len(pcm) < SAMPLE_RATE // 4 * 2 or len(pcm) % 2:
            raise ValueError("Âm thanh quá ngắn hoặc không hợp lệ")
        if not any(pcm):
            raise ValueError("Âm thanh trống")
        folder = self.data_dir / "raw" / label / group_id
        folder.mkdir(parents=True, exist_ok=True)
        number = 1
        while True:
            dest = folder / f"{number:03d}.wav"
            try:
                with dest.open("xb") as fh:
                    with wave.open(fh, "wb") as wav:
                        wav.setnchannels(1)
                        wav.setsampwidth(2)
                        wav.setframerate(SAMPLE_RATE)
                        wav.writeframes(pcm)
                return dest
            except FileExistsError:
                number += 1
            except Exception:
                dest.unlink(missing_ok=True)
                raise

    def count(self, label, group_id):
        return len(list((self.data_dir / "raw" / label / group_id).glob("*.wav")))
