#!/usr/bin/env python3
"""
Script quét tự động các cửa sổ hội thoại (conversation sessions) đang hoạt động
trong thư mục Antigravity brain để phục vụ đồng bộ đa agent.
"""

import os
import json
import datetime
from pathlib import Path

def scan_sessions(max_results=8):
    home = Path.home()
    brain_dir = home / ".gemini" / "antigravity" / "brain"
    
    if not brain_dir.exists():
        print(f"[!] Không tìm thấy thư mục brain: {brain_dir}")
        return

    convs = []
    for entry in os.scandir(brain_dir):
        if entry.is_dir() and entry.name != "tempmediaStorage":
            t_path = Path(entry.path) / ".system_generated" / "logs" / "transcript.jsonl"
            if t_path.exists():
                mtime = t_path.stat().st_mtime
                convs.append((mtime, entry.name, t_path))

    convs.sort(reverse=True, key=lambda x: x[0])

    print("=" * 80)
    print(" DANH SÁCH CÁC CỬA SỔ AGENT ĐANG HOẠT ĐỘNG TRÊN HỆ THỐNG")
    print("=" * 80)

    for idx, (mtime, cid, path) in enumerate(convs[:max_results], 1):
        dt = datetime.datetime.fromtimestamp(mtime).strftime("%H:%M:%S %d/%m/%Y")
        last_prompt = ""
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    data = json.loads(line)
                    if data.get("type") == "USER_INPUT":
                        content = data.get("content", "")
                        if isinstance(content, str):
                            # Clean up XML tags for readability
                            clean_text = content.replace("<USER_REQUEST>", "").replace("</USER_REQUEST>", "")
                            idx_meta = clean_text.find("<ADDITIONAL_METADATA>")
                            if idx_meta != -1:
                                clean_text = clean_text[:idx_meta]
                            last_prompt = clean_text.strip().replace("\n", " ")
        except Exception as e:
            last_prompt = f"(Lỗi đọc transcript: {e})"

        print(f"[{idx}] ID: {cid} | Cập nhật lần cuối: {dt}")
        print(f"    Task / Yêu cầu gần nhất: {last_prompt[:130]}...")
        print("-" * 80)

if __name__ == "__main__":
    scan_sessions()
