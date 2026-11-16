#!/usr/bin/env python3
"""Export local Souta KWS recordings into a *private* shared dataset clone.

Keeps speaker identity consistent across labels, namespaces recorder-generated
speaker/session IDs with a stable contributor ID, and never exports the local
registry containing human-readable names. Does not commit, push, or delete.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import sys
import wave
from pathlib import Path

LABELS = ("positive", "negative", "hard_negative", "ambient")
SOURCE_DEFAULT = Path(__file__).resolve().parent / "data" / "raw"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for piece in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(piece)
    return digest.hexdigest()


def validate_wav(path: Path) -> None:
    try:
        with wave.open(str(path), "rb") as source:
            props = (source.getnchannels(), source.getsampwidth(), source.getframerate())
            frames = source.getnframes()
    except (wave.Error, EOFError, OSError) as exc:
        raise ValueError(f"Không đọc được WAV {path}: {exc}") from exc
    if props != (1, 2, 16000) or frames < 4000:
        raise ValueError(f"WAV không đạt 16 kHz mono PCM 16-bit / >=0.25s: {path}")


def export_recordings(source_dir: Path, dest_dir: Path, collector: str) -> tuple[int, int]:
    if not re.fullmatch(r"[a-z][a-z0-9]{1,11}", collector):
        raise ValueError("Mã cộng tác viên phải gồm 2–12 ký tự a-z/0-9, bắt đầu bằng chữ; VD: c01")
    if not source_dir.is_dir():
        raise ValueError(f"Không tìm thấy thư mục thu âm: {source_dir}")
    # Never allow exporting onto the original source tree.
    source_abs = source_dir.resolve()
    dest_abs = dest_dir.resolve()
    if source_abs == dest_abs or source_abs in dest_abs.parents or dest_abs in source_abs.parents:
        raise ValueError("Thư mục đích phải độc lập với thư mục ghi âm nguồn")

    jobs: list[tuple[Path, Path]] = []
    for label in LABELS:
        for speaker_dir in sorted((source_dir / label).glob("*")):
            if not speaker_dir.is_dir():
                continue
            expected = "session" if label == "ambient" else "speaker"
            match = re.fullmatch(rf"{expected}_(\d{{3,}})", speaker_dir.name)
            if not match:
                raise ValueError(f"Tên nhóm không hợp lệ tại {speaker_dir}")
            target_group = f"{expected}_{collector}_{match.group(1)}"
            for wav in sorted(speaker_dir.glob("*.wav")):
                if wav.is_symlink():
                    raise ValueError(f"Không xuất symlink: {wav}")
                validate_wav(wav)
                jobs.append((wav, dest_dir / label / target_group / wav.name))

    if not jobs:
        raise ValueError("Chưa có file WAV trong data/raw. Hãy ghi âm và lưu mẫu trước.")

    # Detect conflicting existing filenames before writing anything.
    skipped = 0
    pending: list[tuple[Path, Path]] = []
    for src, dst in jobs:
        if dst.exists():
            if not dst.is_file() or sha256(src) != sha256(dst):
                raise FileExistsError(f"File đích đã tồn tại nhưng khác nội dung: {dst}. Không ghi đè!")
            skipped += 1
        else:
            pending.append((src, dst))

    copied = 0
    for src, dst in pending:
        dst.parent.mkdir(parents=True, exist_ok=True)
        try:
            with src.open("rb") as infile, dst.open("xb") as outfile:
                shutil.copyfileobj(infile, outfile)
        except FileExistsError:
            if sha256(src) == sha256(dst):
                skipped += 1
                continue
            raise FileExistsError(f"File đích xuất hiện và khác dữ liệu: {dst}")
        except Exception:
            dst.unlink(missing_ok=True)
            raise
        copied += 1
    return copied, skipped


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collector", required=True, help="Mã duy nhất của người thu: c01, c02, ...")
    parser.add_argument("--source", type=Path, default=SOURCE_DEFAULT, help="Mặc định: training/data/raw")
    parser.add_argument("--dest", type=Path, required=True, help="Đường dẫn đến PRIVATE-DATASET-REPO/data/raw")
    args = parser.parse_args(argv)
    try:
        copied, skipped = export_recordings(args.source, args.dest, args.collector)
    except (ValueError, FileExistsError, OSError) as exc:
        print(f"[souta-export] ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Xuất thành công: {copied} file mới, {skipped} file đã có (giống nội dung).")
    print(f"Thư mục: {args.dest.resolve()}")
    print("Chưa upload tự động. Chỉ git push sau khi xác nhận repository dataset là PRIVATE.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
