#!/usr/bin/env python3
"""Create a ZIP containing only labeled WAV files (not speaker real names)."""
from __future__ import annotations
import argparse
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZIP_DEFLATED, ZipFile
from export_dataset import export_recordings, SOURCE_DEFAULT


def make_dataset_zip(collector, destination, source_dir=SOURCE_DEFAULT):
    destination = Path(destination).expanduser()
    with TemporaryDirectory(prefix="souta_export_") as tmp:
        root = Path(tmp)
        raw = root / "dataset" / "raw"
        export_recordings(Path(source_dir).expanduser(), raw, collector)
        wavs = sorted(raw.rglob("*.wav"))
        if not wavs:
            raise ValueError("Chưa có WAV để xuất.")
        destination.parent.mkdir(parents=True, exist_ok=True)
        created = False
        try:
            with ZipFile(destination, "x", compression=ZIP_DEFLATED) as zipped:
                created = True
                for wav in wavs:
                    zipped.write(wav, wav.relative_to(root).as_posix())
            with ZipFile(destination) as zipped:
                if zipped.testzip() is not None:
                    raise ValueError("File ZIP bị lỗi CRC.")
        except Exception:
            if created:
                destination.unlink(missing_ok=True)
            raise
    return len(wavs), destination


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--collector", required=True, help="Tên/mã người thu data (không dấu, không khoảng trắng), VD: c01 hoặc hoanganh")
    parser.add_argument("--source", type=Path, default=SOURCE_DEFAULT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path(__file__).resolve().parent / "exports" / f"souta_kws_{args.collector}_{stamp}.zip"
    try:
        count, path = make_dataset_zip(args.collector, output, args.source)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"Lỗi tạo ZIP: {exc}\n")
    print(f"Đã xuất {count} WAV theo nhãn: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
