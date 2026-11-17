#!/usr/bin/env python3
"""Standalone SOUTA wakeword data recorder for Windows and Ubuntu."""
from __future__ import annotations

import json
import os
import queue
import re
import tempfile
import threading
import time
import wave
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import sounddevice as sd
from export_zip import make_dataset_zip

SAMPLE_RATE = 16000
DATA = Path(__file__).resolve().parent / "data"
LABELS = ("positive", "negative", "hard_negative", "ambient")
DESCRIPTIONS = {
    "positive": 'Có từ khóa: "Sô ta", "Sô ta ơi"',
    "negative": "Câu nói không có từ khóa",
    "hard_negative": 'Gần âm: "sổ tay", "cô ta"',
    "ambient": "Tiếng ồn môi trường, không có từ khóa",
}


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


def microphone_id(label):
    if label == "Mặc định":
        return None
    match = re.match(r"^(\d+): ", label)
    return int(match.group(1)) if match else None


def capture_pcm(duration, device, cancel, stop):
    chunks = []
    glitches = []

    def callback(indata, frames, time_info, status):
        if status:
            glitches.append(str(status))
        if not cancel.is_set():
            chunks.append(bytes(indata))

    with sd.RawInputStream(samplerate=SAMPLE_RATE, channels=1, dtype="int16",
                           blocksize=1024, device=device, callback=callback):
        deadline = time.monotonic() + duration
        while not cancel.is_set() and not stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.03, remaining))
    if cancel.is_set():
        return None
    raw = b"".join(chunks)[:int(duration * SAMPLE_RATE) * 2]
    if len(raw) < SAMPLE_RATE // 4 * 2:
        raise ValueError("Micro ghi dưới 0,25 giây")
    if glitches:
        raise RuntimeError("Micro bị mất mẫu âm thanh: " + "; ".join(sorted(set(glitches))))
    return raw


class RecorderApp:
    def __init__(self, root):
        self.root = root
        self.store = RecorderStore()
        self.label = tk.StringVar(value="positive")
        self.person_name = tk.StringVar()
        self.member = tk.StringVar()
        self.duration = tk.StringVar(value="3")
        self.device = tk.StringVar(value="Mặc định")
        self.collector = tk.StringVar(value="c01")
        self.status = tk.StringVar(value="Sẵn sàng. Chọn nhãn và người nói.")
        self.counter = tk.StringVar(value="")
        self.state = "idle"
        self.preview = None
        self.cancel = None
        self.stop = None
        self.events = queue.Queue()
        self.started = 0.0
        self.rec_duration = 3
        self.member_map = {}
        self.closed = False
        self.setup_ui()
        self.refresh_members()
        self.set_buttons()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.poll)

    def setup_ui(self):
        self.root.title("SOUTA · Thu thập dữ liệu giọng nói")
        self.root.geometry("760x690")
        self.root.minsize(650, 600)
        body = ttk.Frame(self.root, padding=16)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="SOUTA DATA RECORDER", font=("Arial", 16, "bold")).pack(anchor="w")
        ttk.Label(body, text="Ghi âm → nghe lại → lưu/bỏ → xuất ZIP có nhãn").pack(anchor="w", pady=(5, 12))

        group = ttk.LabelFrame(body, text="1 · Nhãn và người nói", padding=10)
        group.pack(fill="x", pady=5)
        line = ttk.Frame(group)
        line.pack(fill="x")
        ttk.Label(line, text="Nhãn:").pack(side="left")
        self.label_combo = ttk.Combobox(line, textvariable=self.label, values=LABELS,
                                         state="readonly", width=18)
        self.label_combo.pack(side="left", padx=8)
        self.label_combo.bind("<<ComboboxSelected>>", lambda _: self.refresh_members())
        ttk.Label(group, textvariable=self.counter).pack(anchor="w", pady=4)
        line = ttk.Frame(group)
        line.pack(fill="x", pady=4)
        ttk.Label(line, text="Tên nội bộ:").pack(side="left")
        self.name_entry = ttk.Entry(line, textvariable=self.person_name)
        self.name_entry.pack(side="left", padx=8, fill="x", expand=True)
        self.register_btn = ttk.Button(line, text="Đăng ký mới", command=self.register)
        self.register_btn.pack(side="left")
        line = ttk.Frame(group)
        line.pack(fill="x", pady=4)
        ttk.Label(line, text="Người / phiên:").pack(side="left")
        self.member_combo = ttk.Combobox(line, textvariable=self.member, state="readonly")
        self.member_combo.pack(side="left", padx=8, fill="x", expand=True)
        self.member_combo.bind("<<ComboboxSelected>>", lambda _: self.refresh_count())
        ttk.Label(group, text="Tên nội bộ không nằm trong file ZIP gửi người phụ trách.").pack(anchor="w")

        group = ttk.LabelFrame(body, text="2 · Microphone và ghi âm", padding=10)
        group.pack(fill="x", pady=5)
        line = ttk.Frame(group)
        line.pack(fill="x")
        ttk.Label(line, text="Microphone:").pack(side="left")
        available = ["Mặc định"] + [
            f"{i}: {item['name']}" for i, item in enumerate(sd.query_devices())
            if item["max_input_channels"] > 0
        ]
        self.device_combo = ttk.Combobox(line, textvariable=self.device, values=available,
                                          state="readonly")
        self.device_combo.pack(side="left", padx=8, fill="x", expand=True)
        ttk.Label(line, text="Giây:").pack(side="left")
        self.duration_combo = ttk.Combobox(line, textvariable=self.duration,
                                            values=("3", "5", "10", "30", "60"),
                                            width=5, state="readonly")
        self.duration_combo.pack(side="left", padx=8)
        line = ttk.Frame(group)
        line.pack(fill="x", pady=8)
        self.record_btn = ttk.Button(line, text="Bắt đầu ghi", command=self.start)
        self.record_btn.pack(side="left", padx=3)
        self.stop_btn = ttk.Button(line, text="Kết thúc sớm", command=self.stop_early)
        self.stop_btn.pack(side="left", padx=3)
        self.cancel_btn = ttk.Button(line, text="Hủy ghi", command=self.cancel_capture)
        self.cancel_btn.pack(side="left", padx=3)
        self.progress = ttk.Progressbar(group, maximum=100)
        self.progress.pack(fill="x", pady=4)
        line = ttk.Frame(group)
        line.pack(fill="x", pady=5)
        self.play_btn = ttk.Button(line, text="Nghe lại", command=self.play)
        self.play_btn.pack(side="left", padx=3)
        self.save_btn = ttk.Button(line, text="Lưu vào dataset", command=self.save)
        self.save_btn.pack(side="left", padx=3)
        self.discard_btn = ttk.Button(line, text="Bỏ bản thu", command=self.discard)
        self.discard_btn.pack(side="left", padx=3)

        group = ttk.LabelFrame(body, text="3 · Xuất dữ liệu", padding=10)
        group.pack(fill="x", pady=5)
        line = ttk.Frame(group)
        line.pack(fill="x")
        ttk.Label(line, text="Mã cộng tác viên:").pack(side="left")
        ttk.Entry(line, textvariable=self.collector, width=12).pack(side="left", padx=8)
        self.export_btn = ttk.Button(line, text="Xuất ZIP dataset", command=self.export_zip)
        self.export_btn.pack(side="left")
        ttk.Label(group, text="ZIP gồm WAV theo nhãn, không chứa registry tên người nói.").pack(anchor="w", pady=5)

        group = ttk.LabelFrame(body, text="4 · Trạng thái", padding=10)
        group.pack(fill="x", pady=5)
        ttk.Label(group, textvariable=self.status, wraplength=690).pack(anchor="w")

    def current_kind(self):
        return "sessions" if self.label.get() == "ambient" else "speakers"

    def refresh_members(self, selected=None):
        items = self.store.members(self.current_kind())
        self.member_map = {f"{i['name']} · {i['id']}": i["id"] for i in items}
        self.member_combo["values"] = list(self.member_map)
        found = next((key for key, value in self.member_map.items() if value == selected), "")
        if found:
            self.member.set(found)
        elif self.member.get() not in self.member_map:
            self.member.set("")
        self.refresh_count()

    def refresh_count(self):
        label = self.label.get()
        current = self.member_map.get(self.member.get())
        number = self.store.count(label, current) if current else 0
        self.counter.set(DESCRIPTIONS[label] + f" · Đã lưu {number} mẫu")

    def register(self):
        try:
            member = self.store.register(self.current_kind(), self.person_name.get())
        except (ValueError, OSError) as exc:
            messagebox.showerror("Đăng ký thất bại", str(exc))
            return
        self.person_name.set("")
        self.refresh_members(member["id"])

    def set_buttons(self):
        recording = self.state == "recording"
        reviewing = self.state == "reviewing"
        self.record_btn["state"] = "normal" if self.state == "idle" else "disabled"
        self.stop_btn["state"] = "normal" if recording else "disabled"
        self.cancel_btn["state"] = "normal" if recording else "disabled"
        self.play_btn["state"] = "normal" if reviewing else "disabled"
        self.save_btn["state"] = "normal" if reviewing else "disabled"
        self.discard_btn["state"] = "normal" if reviewing else "disabled"
        self.export_btn["state"] = "disabled" if recording else "normal"
        for widget in (self.label_combo, self.member_combo, self.device_combo, self.duration_combo):
            widget["state"] = "disabled" if recording else "readonly"
        self.name_entry["state"] = "disabled" if recording else "normal"
        self.register_btn["state"] = "disabled" if recording else "normal"

    def start(self):
        if self.state != "idle":
            return
        if not self.member_map.get(self.member.get()):
            messagebox.showwarning("Thiếu người nói", "Đăng ký hoặc chọn người nói trước khi ghi.")
            return
        device = microphone_id(self.device.get())
        try:
            sd.check_input_settings(device=device, channels=1, samplerate=SAMPLE_RATE, dtype="int16")
        except (sd.PortAudioError, ValueError) as exc:
            messagebox.showerror("Microphone không dùng được", str(exc))
            return
        self.cancel = threading.Event()
        self.stop = threading.Event()
        self.rec_duration = int(self.duration.get())
        self.started = time.monotonic()
        self.state = "recording"
        self.preview = None
        self.status.set("Đang ghi, nói đúng nội dung theo nhãn đã chọn…")
        self.set_buttons()

        def worker():
            try:
                result = capture_pcm(self.rec_duration, device, self.cancel, self.stop)
                self.events.put(("audio", result))
            except Exception as exc:
                self.events.put(("error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def stop_early(self):
        if self.state == "recording":
            self.stop.set()

    def cancel_capture(self):
        if self.state == "recording":
            self.cancel.set()

    def poll(self):
        if self.closed:
            return
        if self.state == "recording":
            elapsed = min(self.rec_duration, time.monotonic() - self.started)
            self.progress["value"] = 100 * elapsed / self.rec_duration
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "error":
                    self.state = "idle"
                    self.status.set("Lỗi ghi âm: " + value)
                    messagebox.showerror("Ghi âm thất bại", value)
                elif value is None:
                    self.state = "idle"
                    self.status.set("Đã hủy ghi, không lưu.")
                else:
                    self.preview = value
                    self.state = "reviewing"
                    arr = np.frombuffer(value, dtype="<i2").astype(np.float64)
                    rms = int(np.sqrt(np.mean(arr * arr)))
                    peak = int(np.max(np.abs(arr)))
                    notes = " · CẢNH BÁO: peak cao" if peak >= 32700 else ""
                    self.status.set(
                        f"Đã thu {len(value) / (SAMPLE_RATE * 2):.2f} giây; "
                        f"RMS {rms}, peak {peak}{notes}. Nghe và quyết định lưu/bỏ."
                    )
                self.set_buttons()
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def play(self):
        if self.preview is None:
            return
        try:
            sd.stop()
            sd.play(np.frombuffer(self.preview, dtype="<i2"), SAMPLE_RATE, blocking=False)
        except Exception as exc:
            messagebox.showerror("Lỗi phát âm thanh", str(exc))

    def discard(self):
        if self.state == "reviewing":
            sd.stop()
            self.preview = None
            self.state = "idle"
            self.progress["value"] = 0
            self.status.set("Đã bỏ bản thu.")
            self.set_buttons()

    def save(self):
        if self.state != "reviewing":
            return
        person_id = self.member_map.get(self.member.get())
        if not person_id:
            messagebox.showwarning("Chưa chọn người nói", "Cần chọn người/phiên trước khi lưu.")
            return
        try:
            path = self.store.save_pcm(self.label.get(), person_id, self.preview)
        except Exception as exc:
            messagebox.showerror("Không thể lưu", str(exc))
            return
        sd.stop()
        self.preview = None
        self.state = "idle"
        self.progress["value"] = 0
        self.status.set("Đã lưu: " + str(path.relative_to(self.store.data_dir)))
        self.refresh_count()
        self.set_buttons()

    def export_zip(self):
        code = self.collector.get().strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9]{1,11}", code):
            messagebox.showerror("Mã không hợp lệ", "Mã 2–12 ký tự a-z, 0-9, bắt đầu bằng chữ.")
            return
        suggested = f"souta_kws_{code}_{datetime.now():%Y%m%d_%H%M%S}.zip"
        filename = filedialog.asksaveasfilename(
            title="Lưu file ZIP gửi người phụ trách", initialfile=suggested,
            defaultextension=".zip", filetypes=[("ZIP", "*.zip")])
        if not filename:
            return
        try:
            count, output = make_dataset_zip(code, Path(filename), self.store.data_dir / "raw")
        except (OSError, ValueError) as exc:
            messagebox.showerror("Xuất ZIP thất bại", str(exc))
            return
        self.status.set(f"Đã đóng gói {count} file WAV: {output}")
        messagebox.showinfo("Đã tạo ZIP", f"Đã xuất {count} WAV theo nhãn.\n{output}\n\n"
                            "Gửi ZIP qua kênh riêng tư, không gửi registry.")

    def close(self):
        if self.state in ("recording", "reviewing"):
            if not messagebox.askyesno("Thoát", "Hủy bản thu chưa lưu và thoát?"):
                return
        self.closed = True
        if self.cancel:
            self.cancel.set()
        sd.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        RecorderApp(root)
    except Exception as exc:
        messagebox.showerror("Không mở được Recorder", str(exc))
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
