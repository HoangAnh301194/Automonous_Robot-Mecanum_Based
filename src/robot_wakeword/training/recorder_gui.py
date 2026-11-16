#!/usr/bin/env python3
"""Standalone desktop dataset recorder for Souta KWS (no ROS dependency).

- Register speakers/sessions locally; private names stay in data/recorder_registry.json.
- Record -> review/listen -> explicit Save or Discard. Never auto-label or auto-save.
- Linux PulseAudio parec/paplay; 16 kHz mono 16-bit PCM WAV.
- No third-party Python dependencies; Tkinter is supplied by python3-tk.
"""
from __future__ import annotations

import array
import math
import json
import os
import queue
import re
import shutil
import subprocess
import tempfile
import sys
import threading
import time
import tkinter as tk
import wave
from pathlib import Path
from tkinter import messagebox, ttk

SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2
LABELS = {
    "Có từ khóa — positive (1)": "positive",
    "Không có từ khóa — negative (0)": "negative",
    "Gần giống từ khóa — hard_negative (0)": "hard_negative",
    "Tiếng môi trường — ambient (0)": "ambient",
}
DURATIONS = ("3", "5", "10", "30", "60")
DATA_DIR = Path(__file__).resolve().parent / "data"


class RecorderStore:
    """Pure filesystem logic; no UI or microphone access."""

    def __init__(self, data_dir=DATA_DIR):
        self.data_dir = Path(data_dir)
        self.registry_path = self.data_dir / "recorder_registry.json"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        if self.registry_path.exists():
            data = json.loads(self.registry_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or not all(isinstance(data.get(k), list) for k in ("speakers", "sessions")):
                raise ValueError("Invalid recorder_registry.json")
            self.registry = data
        else:
            self.registry = {"version": 1, "speakers": [], "sessions": []}

    @staticmethod
    def kind_for(label):
        return "sessions" if label == "ambient" else "speakers"

    def members(self, kind):
        if kind not in ("speakers", "sessions"):
            raise ValueError("Invalid group kind")
        return list(self.registry[kind])

    def register(self, kind, name):
        if kind not in ("speakers", "sessions"):
            raise ValueError("Invalid group kind")
        name = " ".join(name.strip().split())
        if not name or len(name) > 80 or any(ord(c) < 32 for c in name):
            raise ValueError("Nhập tên từ 1 đến 80 ký tự hợp lệ.")
        if any(x["name"].casefold() == name.casefold() for x in self.registry[kind]):
            raise ValueError("Tên đã được đăng ký; hãy chọn trong danh sách.")
        prefix = "session" if kind == "sessions" else "speaker"
        used = {x["id"] for x in self.registry[kind]}
        next_n = 1
        while f"{prefix}_{next_n:03d}" in used:
            next_n += 1
        item = {"id": f"{prefix}_{next_n:03d}", "name": name}
        self.registry[kind].append(item)
        self._save_registry()
        return item

    def _save_registry(self):
        # Atomic registry replacement, so an interrupted write does not corrupt metadata.
        fd, filename = tempfile.mkstemp(prefix=".registry_", suffix=".json", dir=self.data_dir)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as out:
                json.dump(self.registry, out, ensure_ascii=False, indent=2)
                out.write("\n")
                out.flush()
                os.fsync(out.fileno())
            os.replace(filename, self.registry_path)
        finally:
            Path(filename).unlink(missing_ok=True)

    def save_preview(self, preview, label, group_id):
        """Copy a reviewed WAV into the corpus using an exclusive, never-overwrite filename."""
        if label not in LABELS.values():
            raise ValueError("Invalid label")
        kind = self.kind_for(label)
        if group_id not in {r["id"] for r in self.registry[kind]}:
            raise ValueError("Chưa chọn người nói / phiên thu hợp lệ")
        expected_id = r"session_\d{3,}" if kind == "sessions" else r"speaker_\d{3,}"
        if not re.fullmatch(expected_id, group_id):
            raise ValueError("Invalid group identifier")
        preview = Path(preview)
        if not preview.is_file():
            raise FileNotFoundError(preview)
        # Reject silent/invalid WAV before including it in training data.
        with wave.open(str(preview), "rb") as wav:
            if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) != (1, 2, SAMPLE_RATE):
                raise ValueError("File phải là WAV 16 kHz, mono, PCM 16-bit")
            raw = wav.readframes(wav.getnframes())
        if len(raw) < SAMPLE_RATE // 4 * SAMPLE_WIDTH or pcm_metrics(raw)[0] == 0:
            raise ValueError("Bản thu trống hoặc quá ngắn; hãy thu lại.")
        folder = self.data_dir / "raw" / label / group_id
        folder.mkdir(parents=True, exist_ok=True)
        used = [int(p.stem) for p in folder.glob("*.wav") if re.fullmatch(r"\d+", p.stem)]
        number = max(used, default=0) + 1
        while True:
            destination = folder / f"{number:03d}.wav"
            try:
                with destination.open("xb") as out, preview.open("rb") as source:
                    shutil.copyfileobj(source, out)
                    out.flush()
                    os.fsync(out.fileno())
                return destination
            except FileExistsError:
                number += 1
            except Exception:
                destination.unlink(missing_ok=True)
                raise

    def count(self, label, group_id):
        return len(list((self.data_dir / "raw" / label / group_id).glob("*.wav")))


def pcm_metrics(raw):
    """Return (RMS, peak) for little-endian S16 PCM without deprecated audioop."""
    if len(raw) % SAMPLE_WIDTH:
        raise ValueError("Dữ liệu PCM 16-bit không đầy đủ")
    samples = array.array("h")
    samples.frombytes(raw)
    if sys.byteorder != "little":
        samples.byteswap()
    if not samples:
        return 0, 0
    return int(math.sqrt(sum(s * s for s in samples) / len(samples))), max(abs(s) for s in samples)


def record_pcm(duration, cancel_event, stop_event, source="@DEFAULT_SOURCE@"):
    """Capture raw audio away from the Tk event thread."""
    cmd = ["parec", "-d", source, "--rate=16000", "--channels=1",
           "--format=s16le", "--raw"]
    process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    blocks = []
    remaining = round(duration * SAMPLE_RATE) * SAMPLE_WIDTH
    try:
        while remaining > 0 and not cancel_event.is_set() and not stop_event.is_set():
            chunk = process.stdout.read(min(4096, remaining))
            if not chunk:
                break
            blocks.append(chunk)
            remaining -= len(chunk)
        raw = b"".join(blocks)
        if cancel_event.is_set():
            return None
        if len(raw) < SAMPLE_RATE // 4 * SAMPLE_WIDTH:
            raise RuntimeError("Micro không ghi đủ 0,25 giây. Kiểm tra nguồn PulseAudio.")
        return raw
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def write_preview(data_dir, pcm):
    """Create an ignored temporary WAV. Only Save copies it into data/raw/."""
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    fd, path = tempfile.mkstemp(prefix=".souta_preview_", suffix=".wav", dir=data_dir)
    os.close(fd)
    try:
        with wave.open(path, "wb") as wav:
            wav.setnchannels(CHANNELS)
            wav.setsampwidth(SAMPLE_WIDTH)
            wav.setframerate(SAMPLE_RATE)
            wav.writeframes(pcm)
    except Exception:
        Path(path).unlink(missing_ok=True)
        raise
    return Path(path)


class SoutaRecorderApp:
    def __init__(self, root):
        self.root = root
        self.store = RecorderStore()
        self.root.title("Souta KWS · Thu thập dữ liệu")
        self.root.geometry("760x610")
        self.root.minsize(660, 560)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.label_text = tk.StringVar(value=next(iter(LABELS)))
        self.name_text = tk.StringVar()
        self.member_text = tk.StringVar()
        self.duration_text = tk.StringVar(value="3")
        self.source_text = tk.StringVar(value="@DEFAULT_SOURCE@")
        self.status_text = tk.StringVar(value="Sẵn sàng. Đăng ký người nói và chọn nhãn.")
        self.meta_text = tk.StringVar(value="Chưa có bản ghi chờ lưu")
        self.counter_text = tk.StringVar()
        self.progress_text = tk.StringVar(value="00:00 / 00:03")
        self.members_map = {}
        self.state = "idle"  # idle / recording / reviewing
        self.preview = None
        self.capture_queue = queue.Queue()
        self.cancel_event = None
        self.stop_event = None
        self.capture_started = 0.0
        self.capture_duration = 3
        self.capture_label = None
        self.capture_group = None
        self.player = None
        self.closing = False
        self._build()
        self._update_member_list()
        self._update_buttons()
        self.root.after(100, self._poll)

    def _build(self):
        style = ttk.Style()
        style.configure("Header.TLabel", font=("Sans", 16, "bold"))
        style.configure("Section.TLabel", font=("Sans", 11, "bold"))
        root_frame = ttk.Frame(self.root, padding=18)
        root_frame.pack(fill="both", expand=True)
        ttk.Label(root_frame, text="SOUTA / SÔ TA · Dataset Recorder", style="Header.TLabel").pack(anchor="w")
        ttk.Label(root_frame, text="Ghi âm → nghe kiểm tra → Lưu hoặc Bỏ. Không tự động đưa file vào dataset.").pack(anchor="w", pady=(3, 14))

        group = ttk.LabelFrame(root_frame, text="1 · Người nói / phiên môi trường", padding=12)
        group.pack(fill="x", pady=(0, 10))
        row = ttk.Frame(group)
        row.pack(fill="x")
        ttk.Label(row, text="Tên đăng ký:").pack(side="left")
        self.name_entry = ttk.Entry(row, textvariable=self.name_text, width=28)
        self.name_entry.pack(side="left", padx=8, fill="x", expand=True)
        self.register_button = ttk.Button(row, text="Đăng ký mới", command=self.register_member)
        self.register_button.pack(side="left")
        row2 = ttk.Frame(group)
        row2.pack(fill="x", pady=(10, 0))
        ttk.Label(row2, text="Chọn người/phiên:").pack(side="left")
        self.member_combo = ttk.Combobox(row2, textvariable=self.member_text, state="readonly", width=42)
        self.member_combo.pack(side="left", padx=8, fill="x", expand=True)
        self.member_combo.bind("<<ComboboxSelected>>", lambda _: self._update_count())
        ttk.Label(group, text="Tên thật chỉ lưu cục bộ trong data/recorder_registry.json; file âm thanh sử dụng speaker_001, ...").pack(anchor="w", pady=(7, 0))

        setup = ttk.LabelFrame(root_frame, text="2 · Nhãn và thời lượng", padding=12)
        setup.pack(fill="x", pady=(0, 10))
        row = ttk.Frame(setup)
        row.pack(fill="x")
        ttk.Label(row, text="Nhãn:").pack(side="left")
        self.label_combo = ttk.Combobox(row, textvariable=self.label_text, values=list(LABELS),
                                        state="readonly", width=44)
        self.label_combo.pack(side="left", padx=8, fill="x", expand=True)
        self.label_combo.bind("<<ComboboxSelected>>", self._on_label_change)
        ttk.Label(row, text="Giây:").pack(side="left")
        self.duration_combo = ttk.Combobox(row, textvariable=self.duration_text, values=DURATIONS,
                                            width=5, state="readonly")
        self.duration_combo.pack(side="left", padx=(8, 0))
        row3 = ttk.Frame(setup)
        row3.pack(fill="x", pady=(9, 0))
        ttk.Label(row3, text="Nguồn micro:").pack(side="left")
        self.source_entry = ttk.Entry(row3, textvariable=self.source_text)
        self.source_entry.pack(side="left", padx=8, fill="x", expand=True)
        ttk.Label(row3, text="Mặc định: @DEFAULT_SOURCE@").pack(side="left")

        capture = ttk.LabelFrame(root_frame, text="3 · Thu âm và duyệt bản ghi", padding=12)
        capture.pack(fill="x", pady=(0, 10))
        self.progress = ttk.Progressbar(capture, maximum=100, value=0)
        self.progress.pack(fill="x", pady=(3, 5))
        ttk.Label(capture, textvariable=self.progress_text).pack(anchor="e")
        row = ttk.Frame(capture)
        row.pack(fill="x", pady=(8, 6))
        self.record_button = ttk.Button(row, text="● Bắt đầu ghi", command=self.start_record)
        self.record_button.pack(side="left", padx=(0, 6))
        self.stop_button = ttk.Button(row, text="■ Kết thúc sớm", command=self.stop_record)
        self.stop_button.pack(side="left", padx=(0, 6))
        self.cancel_button = ttk.Button(row, text="Hủy ghi", command=self.cancel_record)
        self.cancel_button.pack(side="left")
        row = ttk.Frame(capture)
        row.pack(fill="x", pady=(6, 8))
        self.listen_button = ttk.Button(row, text="Nghe lại", command=self.play_preview)
        self.listen_button.pack(side="left", padx=(0, 6))
        self.save_button = ttk.Button(row, text="Lưu vào dataset", command=self.save_record)
        self.save_button.pack(side="left", padx=(0, 6))
        self.discard_button = ttk.Button(row, text="Bỏ bản thu / Không lưu", command=self.discard_record)
        self.discard_button.pack(side="left")
        ttk.Label(capture, textvariable=self.meta_text, wraplength=690).pack(anchor="w", pady=(6, 0))

        summary = ttk.LabelFrame(root_frame, text="4 · Trạng thái", padding=12)
        summary.pack(fill="both", expand=True)
        ttk.Label(summary, textvariable=self.status_text, wraplength=690).pack(anchor="w")
        ttk.Label(summary, textvariable=self.counter_text).pack(anchor="w", pady=(8, 0))
        ttk.Label(summary, text="Positive: nói 'Sô ta' hoặc 'Sô ta ơi'. Hard negative: ví dụ 'sổ tay'. Ambient: tiếng ồn dài.", wraplength=690).pack(anchor="w", pady=(8, 0))

    def _label(self):
        return LABELS[self.label_text.get()]

    def _member_kind(self):
        return self.store.kind_for(self._label())

    def _update_member_list(self, selected_id=None):
        self.members_map = {f"{x['name']}  ·  {x['id']}": x["id"] for x in self.store.members(self._member_kind())}
        self.member_combo["values"] = list(self.members_map)
        selected = next((k for k, v in self.members_map.items() if v == selected_id), None)
        if selected is not None:
            self.member_text.set(selected)
        elif self.member_text.get() not in self.members_map:
            self.member_text.set("")
        self._update_count()

    def _update_count(self):
        group_id = self.members_map.get(self.member_text.get())
        if group_id:
            self.counter_text.set(f"Đã lưu {self.store.count(self._label(), group_id)} file cho nhãn {self._label()} / {group_id}")
        else:
            self.counter_text.set("Chọn một người nói / phiên môi trường để ghi âm.")

    def _on_label_change(self, _event=None):
        self._update_member_list()
        self._update_review_label()

    def _update_review_label(self):
        if self.state == "reviewing" and self.preview:
            self.meta_text.set(f"BẢN THU CHỜ DUYỆT — nhãn lưu: {self._label()}, người/phiên: {self.members_map.get(self.member_text.get(), 'CHƯA CHỌN')}. Hãy nghe và chọn Lưu hoặc Bỏ.")

    def register_member(self):
        try:
            item = self.store.register(self._member_kind(), self.name_text.get())
        except (ValueError, OSError, json.JSONDecodeError) as exc:
            messagebox.showerror("Không thể đăng ký", str(exc))
            return
        self.name_text.set("")
        self._update_member_list(selected_id=item["id"])
        self.status_text.set(f"Đã đăng ký {item['name']} với mã {item['id']} (chỉ lưu cục bộ).")

    def _update_buttons(self):
        recording = self.state == "recording"
        preview = self.state == "reviewing"
        self.record_button["state"] = "normal" if self.state == "idle" else "disabled"
        self.stop_button["state"] = "normal" if recording else "disabled"
        self.cancel_button["state"] = "normal" if recording else "disabled"
        self.listen_button["state"] = "normal" if preview else "disabled"
        self.save_button["state"] = "normal" if preview else "disabled"
        self.discard_button["state"] = "normal" if preview else "disabled"
        for widget in (self.name_entry, self.register_button, self.label_combo,
                       self.member_combo, self.duration_combo, self.source_entry):
            widget["state"] = "disabled" if recording else ("readonly" if isinstance(widget, ttk.Combobox) else "normal")

    def start_record(self):
        if self.state != "idle":
            return
        if not shutil.which("parec"):
            messagebox.showerror("Thiếu parec", "Cài PulseAudio utilities (pulseaudio-utils) để dùng microphone.")
            return
        if not self.members_map.get(self.member_text.get()):
            messagebox.showwarning("Chưa chọn người nói", "Đăng ký hoặc chọn người nói / phiên môi trường trước.")
            return
        self.capture_duration = int(self.duration_text.get())
        self.capture_started = time.monotonic()
        self.cancel_event = threading.Event()
        self.stop_event = threading.Event()
        self.capture_label = self._label()
        self.capture_group = self.members_map[self.member_text.get()]
        self.state = "recording"
        self.meta_text.set(f"ĐANG THU — {self.capture_label} / {self.capture_group}")
        self.status_text.set("Đang ghi âm… nói một lần với positive, hoặc thu tiếng nói/âm nền phù hợp với nhãn.")
        self._update_buttons()
        args = (self.capture_duration, self.cancel_event, self.stop_event, self.source_text.get())
        threading.Thread(target=self._capture_worker, args=args, daemon=True).start()

    def _capture_worker(self, duration, cancel_event, stop_event, source):
        try:
            raw = record_pcm(duration, cancel_event, stop_event, source)
            self.capture_queue.put(("recorded", raw))
        except Exception as exc:
            self.capture_queue.put(("error", str(exc)))

    def stop_record(self):
        if self.state == "recording":
            self.stop_event.set()
            self.status_text.set("Đang kết thúc bản ghi…")

    def cancel_record(self):
        if self.state == "recording":
            self.cancel_event.set()
            self.status_text.set("Đang hủy ghi âm; dữ liệu sẽ không được lưu.")

    def _poll(self):
        if self.closing:
            return
        if self.state == "recording":
            passed = min(time.monotonic() - self.capture_started, self.capture_duration)
            self.progress["value"] = round(100 * passed / self.capture_duration)
            self.progress_text.set(f"{passed:04.1f} / {self.capture_duration:.1f} giây")
        try:
            while True:
                kind, payload = self.capture_queue.get_nowait()
                if kind == "error":
                    self.state = "idle"
                    self.status_text.set(f"Lỗi ghi âm: {payload}")
                    self.meta_text.set("Chưa có bản ghi chờ lưu")
                    messagebox.showerror("Ghi âm thất bại", payload)
                elif payload is None:
                    self.state = "idle"
                    self.status_text.set("Đã hủy ghi âm. Không có file nào được lưu.")
                    self.meta_text.set("Chưa có bản ghi chờ lưu")
                else:
                    self.preview = write_preview(self.store.data_dir, payload)
                    self.state = "reviewing"
                    duration = len(payload) / (SAMPLE_RATE * SAMPLE_WIDTH)
                    rms, peak = pcm_metrics(payload)
                    flags = []
                    if rms < 100:
                        flags.append("âm lượng nhỏ")
                    if peak >= 32700:
                        flags.append("có thể bị clipping")
                    warning = f"; CẢNH BÁO: {', '.join(flags)}" if flags else ""
                    self.status_text.set(f"Đã ghi {duration:.2f} giây · RMS {rms} · Peak {peak}{warning}. Nghe lại trước khi lưu.")
                    self._update_review_label()
                self._update_buttons()
        except queue.Empty:
            pass
        except Exception as exc:
            self.state = "idle"
            self._delete_preview()
            self._update_buttons()
            messagebox.showerror("Không thể xử lý bản thu", str(exc))
        self.root.after(100, self._poll)

    def _stop_playback(self):
        if self.player is not None and self.player.poll() is None:
            self.player.terminate()
        self.player = None

    def play_preview(self):
        if not self.preview or not self.preview.is_file():
            return
        if not shutil.which("paplay"):
            messagebox.showerror("Thiếu paplay", "Máy chưa có paplay (pulseaudio-utils).")
            return
        self._stop_playback()
        try:
            self.player = subprocess.Popen(["paplay", str(self.preview)],
                                           stdin=subprocess.DEVNULL,
                                           stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL)
            self.status_text.set("Đang phát lại bản ghi để kiểm tra chất lượng…")
        except OSError as exc:
            messagebox.showerror("Không phát được", str(exc))

    def _delete_preview(self):
        self._stop_playback()
        if self.preview:
            self.preview.unlink(missing_ok=True)
            self.preview = None

    def discard_record(self):
        if self.state != "reviewing":
            return
        self._delete_preview()
        self.state = "idle"
        self.meta_text.set("Đã bỏ bản thu. Không thêm file vào dataset.")
        self.status_text.set("Đã bỏ bản thu. Cậu có thể ghi lại ngay.")
        self.progress["value"] = 0
        self._update_buttons()

    def save_record(self):
        if self.state != "reviewing" or not self.preview:
            return
        group_id = self.members_map.get(self.member_text.get())
        if not group_id:
            messagebox.showwarning("Chưa chọn người nói", "Chọn người nói / phiên môi trường trước khi lưu.")
            return
        label = self._label()
        try:
            destination = self.store.save_preview(self.preview, label, group_id)
        except Exception as exc:
            messagebox.showerror("Không lưu được", str(exc))
            return
        self._delete_preview()
        self.state = "idle"
        self.meta_text.set(f"ĐÃ LƯU: {destination.relative_to(self.store.data_dir)}")
        self.status_text.set("Đã lưu thành công. Có thể ghi mẫu tiếp theo mà không cần chạy lại lệnh.")
        self.progress["value"] = 0
        self._update_count()
        self._update_buttons()

    def close(self):
        if self.state == "recording":
            if not messagebox.askyesno("Đang ghi âm", "Hủy bản ghi đang thu và thoát?"):
                return
            self.cancel_event.set()
        elif self.state == "reviewing":
            if not messagebox.askyesno("Chưa lưu", "Bỏ bản thu chưa lưu và thoát?"):
                return
        self.closing = True
        self._delete_preview()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        SoutaRecorderApp(root)
    except Exception as exc:
        messagebox.showerror("Không khởi động được Souta Recorder", str(exc))
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
