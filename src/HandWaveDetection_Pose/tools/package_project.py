#!/usr/bin/env python3
"""
Script đóng gói tự động dự án HandWaveDetection_Pose thành file ZIP Portable.
Loại bỏ các file tạm, môi trường ảo, cache và dữ liệu đầu ra không cần thiết.
"""

import os
import sys
import zipfile
from pathlib import Path
from datetime import datetime

# Xử lý encoding stdout cho Windows terminal
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# Các thư mục và file loại trừ không đóng gói
EXCLUDE_DIRS = {
    ".venv",
    "venv",
    "env",
    "__pycache__",
    ".git",
    ".github",
    ".vscode",
    ".idea",
    "outputs",
    "dist",
    ".pytest_cache",
}

EXCLUDE_FILES = {
    ".DS_Store",
    "*.pyc",
    "*.pyo",
    "*.pyd",
    "*.swp",
}

EXCLUDE_EXTENSIONS = {".pyc", ".pyo", ".pyd", ".zip", ".tar.gz", ".log"}


def should_exclude_path(path: Path, root_dir: Path) -> bool:
    rel_path = path.relative_to(root_dir)
    
    # Kiểm tra thư mục bị loại trừ
    for part in rel_path.parts:
        if part in EXCLUDE_DIRS:
            return True
            
    # Kiểm tra file bị loại trừ
    if path.is_file():
        if path.name in EXCLUDE_FILES or path.suffix in EXCLUDE_EXTENSIONS:
            return True
            
    return False


def clean_old_packages(dist_dir: Path):
    """Dọn dẹp các file ZIP cũ trong thư mục dist."""
    if dist_dir.exists():
        for old_zip in dist_dir.glob("HandWaveDetection_Pose_*.zip"):
            try:
                old_zip.unlink()
                print(f"[*] Deleted old package: {old_zip.name}")
            except Exception as e:
                print(f"[!] Warning: Could not delete {old_zip.name}: {e}")


def package_project():
    project_root = Path(__file__).resolve().parent.parent
    dist_dir = project_root / "dist"
    dist_dir.mkdir(exist_ok=True)
    
    # Dọn dẹp các file zip đóng gói cũ
    clean_old_packages(dist_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_zip_name = f"HandWaveDetection_Pose_{timestamp}.zip"
    output_zip_path = dist_dir / output_zip_name
    
    print(f"[*] Project root: {project_root}")
    print(f"[*] Packaging to: {output_zip_path} ...")
    
    file_count = 0
    total_size = 0
    
    with zipfile.ZipFile(output_zip_path, "w", zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(project_root):
            root_path = Path(root)
            
            if should_exclude_path(root_path, project_root):
                continue
                
            for file in files:
                file_path = root_path / file
                if should_exclude_path(file_path, project_root):
                    continue
                    
                arcname = file_path.relative_to(project_root)
                zipf.write(file_path, arcname)
                file_count += 1
                total_size += file_path.stat().st_size
                print(f"  + Added: {arcname}")
                
    size_mb = total_size / (1024 * 1024)
    zip_size_mb = output_zip_path.stat().st_size / (1024 * 1024)
    
    print("\n[SUCCESS] Project packaged successfully!")
    print(f"    - Total files: {file_count}")
    print(f"    - Raw size: {size_mb:.2f} MB")
    print(f"    - ZIP size: {zip_size_mb:.2f} MB")
    print(f"    - Package file: {output_zip_path}\n")
    return output_zip_path


if __name__ == "__main__":
    package_project()
