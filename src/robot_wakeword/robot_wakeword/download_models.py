#!/usr/bin/env python3
"""Automated, robust downloader for Vosk pretrained models."""
from __future__ import annotations

import argparse
import sys
import time
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

DEFAULT_MODELS = {
    "vosk-model-small-vn-0.4": "https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip",
}


def is_model_valid(model_dir: Path) -> bool:
    """Check if model directory exists and has essential Kaldi/Vosk files."""
    if not model_dir.is_dir():
        return False
    # Standard Vosk models contain am/, conf/, or similar model graph files
    has_files = any(model_dir.iterdir())
    conf_exists = (model_dir / "conf").is_dir() or (model_dir / "am").is_dir() or any(model_dir.glob("*.mdl"))
    return has_files and (conf_exists or (model_dir / "README").exists())


def download_with_progress(url: str, dest_path: Path, max_retries: int = 3, timeout: int = 30) -> bool:
    """Download file with visual progress bar and bounded retries."""
    for attempt in range(1, max_retries + 1):
        try:
            print(f"Downloading {url} (attempt {attempt}/{max_retries})...")
            start_time = time.monotonic()
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req, timeout=timeout) as response:
                total_size = int(response.headers.get("Content-Length", 0))
                downloaded = 0
                block_size = 64 * 1024  # 64 KB
                with open(dest_path, "wb") as out_file:
                    while True:
                        chunk = response.read(block_size)
                        if not chunk:
                            break
                        out_file.write(chunk)
                        downloaded += len(chunk)
                        if total_size > 0:
                            percent = downloaded * 100 / total_size
                            speed_mb = (downloaded / (1024 * 1024)) / max(0.1, (time.monotonic() - start_time))
                            sys.stdout.write(f"\r  [{percent:5.1f}%] {downloaded // 1048576}MB / {total_size // 1048576}MB ({speed_mb:.1f} MB/s)")
                            sys.stdout.flush()
                sys.stdout.write("\n")
            return True
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError) as err:
            print(f"\n[Warning] Download failed: {err}")
            if dest_path.exists():
                dest_path.unlink()
            if attempt < max_retries:
                time.sleep(2 * attempt)
            else:
                print(f"[Error] Failed to download {url} after {max_retries} attempts.")
                return False
    return False


def safe_extract_zip(zip_path: Path, target_dir: Path) -> bool:
    """Extract ZIP file safely without path traversal vulnerabilities."""
    try:
        if not zipfile.is_zipfile(zip_path):
            print(f"[Error] Downloaded file is not a valid ZIP: {zip_path}")
            return False
        with zipfile.ZipFile(zip_path, "r") as zf:
            corrupt = zf.testzip()
            if corrupt is not None:
                print(f"[Error] Corrupted ZIP file detected at element {corrupt}")
                return False
            for member in zf.namelist():
                member_path = (target_dir / member).resolve()
                if not str(member_path).startswith(str(target_dir.resolve())):
                    raise RuntimeError(f"ZipSlip detected in member: {member}")
            zf.extractall(target_dir)
        return True
    except Exception as err:
        print(f"[Error] Extraction failed: {err}")
        return False


def get_models_from_config(config_path: Path | None) -> dict[str, str]:
    """Download the single Vietnamese model declared in the package configuration."""
    from robot_wakeword.probe import DEFAULT_CONFIG_PATH, get_active_keywords, load_config

    config = load_config(config_path or DEFAULT_CONFIG_PATH)
    active = get_active_keywords(config)
    if not active:
        return {}
    name = active["vi_souta"]["model"]
    return {name: DEFAULT_MODELS.get(name, f"https://alphacephei.com/vosk/models/{name}.zip")}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models-dir", type=Path, default=Path.home() / "vosk-models",
                        help="Target directory for extracted models (default: ~/vosk-models)")
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "config" / "keywords.json",
                        help="Path to keywords.json configuration")
    parser.add_argument("--force", action="store_true", help="Force re-download even if model exists")
    args = parser.parse_args(argv)

    models_dir = args.models_dir.expanduser().resolve()
    models_dir.mkdir(parents=True, exist_ok=True)
    models = get_models_from_config(args.config)
    if not models:
        print("[ERROR] No enabled models found in wake-word configuration.", file=sys.stderr)
        return 1

    print(f"Target models directory: {models_dir}")
    all_success = True

    for model_name, url in models.items():
        dest_folder = models_dir / model_name
        if not args.force and is_model_valid(dest_folder):
            print(f"[SKIP] Model '{model_name}' already exists and is valid at: {dest_folder}")
            continue

        temp_zip = models_dir / f"{model_name}.zip.tmp"
        if not download_with_progress(url, temp_zip):
            all_success = False
            continue

        print(f"Extracting {model_name}...")
        if safe_extract_zip(temp_zip, models_dir):
            print(f"[OK] Model '{model_name}' ready at: {dest_folder}")
        else:
            all_success = False
        if temp_zip.exists():
            temp_zip.unlink()

    return 0 if all_success else 1


if __name__ == "__main__":
    sys.exit(main())
