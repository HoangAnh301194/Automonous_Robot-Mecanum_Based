#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audio_player_node.py
Node phát âm thanh MP3 tự động cho Robot (Nav2 Recovery, Mission Control, Chào hỏi):
- Quản lý và phát 7 file âm thanh:
    1. xin_duong.mp3         - Xin đường khi có người chắn
    2. canh_bao_lui.mp3      - Cảnh báo lùi xe (CustomBackUp)
    3. doi_duong.mp3         - Đang tính toán lại đường đi (Replan)
    4. den_nguoi_vay_tay.mp3 - Tiếp cận người vẫy tay
    5. tam_biet_5s.mp3       - Chào tạm biệt sau 5s mời nước
    6. cam_on_phuc_vu.mp3    - Cảm ơn khi hoàn tất phục vụ nước
    7. chuc_20_11.mp3        - Chúc mừng ngày Nhà giáo Việt Nam 20/11
- Lắng nghe:
    + /audio/play_sound (std_msgs/String): phát trực tiếp theo tên file hoặc alias
    + /navigation/speech_request (std_msgs/String): thông điệp từ Nav2 Speak & HumanRecoveryBridge
    + /navigation/operator_alert (diagnostic_msgs/KeyValue): mã alert PRE_BACKUP, REPLANNING, WAITING
    + /mission/status_log (std_msgs/String): sự kiện từ Behavior Tree tuần tra & phục vụ
"""

import os
import queue
import shutil
import subprocess
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')

        self.audio_dir = self._resolve_audio_dir()
        self.get_logger().info(f"📂 Thư mục âm thanh: {self.audio_dir}")

        self.audio_files: Dict[str, str] = {
            'xin_duong': 'xin_duong.mp3',
            'canh_bao_lui': 'canh_bao_lui.mp3',
            'doi_duong': 'doi_duong.mp3',
            'den_nguoi_vay_tay': 'den_nguoi_vay_tay.mp3',
            'tam_biet_5s': 'tam_biet_5s.mp3',
            'cam_on_phuc_vu': 'cam_on_phuc_vu.mp3',
            'chuc_20_11': 'chuc_20_11.mp3',
        }

        # Tìm công cụ phát âm thanh (ưu tiên mpv, sau đó ffplay, sau đó aplay)
        self.player_cmd = self._detect_player_cmd()
        self.get_logger().info(f"🔊 Trình phát âm thanh: {self.player_cmd}")

        self._lock = threading.Lock()
        self._audio_queue: queue.Queue = queue.Queue(maxsize=10)
        self._current_proc: Optional[subprocess.Popen] = None
        self._current_key: Optional[str] = None
        self._last_play_time: Dict[str, float] = {}
        self._cooldown_sec = 2.5  # Không phát lặp cùng 1 file trong 2.5s
        self._shutdown_event = threading.Event()

        # Thread phát âm thanh tuần tự từ hàng đợi
        self._worker_thread = threading.Thread(
            target=self._queue_worker,
            daemon=True,
            name="audio-queue-worker"
        )
        self._worker_thread.start()

        # Subscriptions
        self.sub_direct = self.create_subscription(
            String, '/audio/play_sound', self.cb_play_sound, 10
        )
        self.sub_speech = self.create_subscription(
            String, '/navigation/speech_request', self.cb_speech_request, 10
        )
        self.sub_alert = self.create_subscription(
            KeyValue, '/navigation/operator_alert', self.cb_operator_alert, 10
        )
        self.sub_mission_log = self.create_subscription(
            String, '/mission/status_log', self.cb_mission_log, 10
        )

        self.get_logger().info("✅ AudioPlayerNode (Hàng đợi tuần tự + Delay mở DAC) đã sẵn sàng!")

    def _resolve_audio_dir(self) -> str:
        candidates = []
        try:
            candidates.append(os.path.join(get_package_share_directory('mo_hinh'), 'audio'))
        except PackageNotFoundError:
            pass

        # Đường dẫn source trong workspace
        candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'audio')))
        candidates.append('/home/orin/ros2_ws/src/mo_hinh/audio')
        candidates.append('/home/a/ros2_ws/src/mo_hinh/audio')

        for path in candidates:
            if os.path.isdir(path) and os.path.exists(os.path.join(path, 'xin_duong.mp3')):
                return path

        return candidates[0] if candidates else 'audio'

    def _detect_player_cmd(self) -> str:
        for cmd in ['mpv', 'ffplay']:
            if shutil.which(cmd):
                return cmd
        return 'ffplay'

    def play_sound(self, sound_key: str, force: bool = False, priority: bool = False):
        """
        Phát file âm thanh theo key hoặc filename.
        priority=True: Âm thanh khẩn cấp (cảnh báo lùi), có quyền ngắt âm thanh thông thường đang phát.
        priority=False: Âm thanh thông thường (xin đường, cảm ơn), chờ phát tuần tự, không ngắt câu đang nói.
        """
        clean_key = sound_key.strip().replace('.mp3', '').lower()
        filename = self.audio_files.get(clean_key)
        if not filename:
            filename = f"{clean_key}.mp3" if not clean_key.endswith('.mp3') else clean_key

        filepath = os.path.join(self.audio_dir, filename)
        if not os.path.isfile(filepath):
            self.get_logger().warn(f"Không tìm thấy file âm thanh: {filepath}")
            return

        now = time.monotonic()
        if not force:
            last = self._last_play_time.get(clean_key, 0.0)
            if (now - last) < self._cooldown_sec:
                self.get_logger().debug(f"Bỏ qua phát lặp {clean_key} (cooldown)")
                return

        # Nếu chính file này đang phát dở thì không chen đè lên chính nó
        with self._lock:
            if self._current_key == clean_key and self._current_proc and self._current_proc.poll() is None:
                return

        self._last_play_time[clean_key] = now

        if priority:
            # Khẩn cấp: ngắt câu cũ và xóa hàng đợi các câu không khẩn cấp
            with self._lock:
                if self._current_proc and self._current_proc.poll() is None:
                    try:
                        self._current_proc.terminate()
                    except Exception:
                        pass
                while not self._audio_queue.empty():
                    try:
                        self._audio_queue.get_nowait()
                        self._audio_queue.task_done()
                    except Exception:
                        break
            try:
                self._audio_queue.put_nowait((filepath, clean_key, True))
            except Exception:
                pass
        else:
            # Thông thường: Kiểm tra nếu đã có trong hàng đợi thì không spam thêm
            queued_keys = [item[1] for item in list(self._audio_queue.queue)]
            if clean_key in queued_keys:
                return
            try:
                self._audio_queue.put_nowait((filepath, clean_key, False))
            except Exception:
                pass

    def _queue_worker(self):
        """Worker thread lấy từng file trong hàng đợi phát tuần tự, không chém ngang."""
        while not self._shutdown_event.is_set():
            try:
                item = self._audio_queue.get(timeout=0.2)
            except Exception:
                continue

            filepath, key, is_priority = item

            with self._lock:
                self._current_key = key
                self.get_logger().info(f"▶️ Đang phát âm thanh: [{key}] -> {os.path.basename(filepath)}")

                # Chèn 350ms khoảng lặng ở đầu stream (adelay=350|350)
                # Đảm bảo phần cứng DAC/Amply thức dậy kịp thời, không bao giờ mất chữ đầu!
                if self.player_cmd == 'mpv':
                    cmd = ['mpv', '--no-video', '--really-quiet', '--af=lavfi=[adelay=350|350]', filepath]
                else:
                    cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', '-af', 'adelay=350|350', filepath]

                try:
                    self._current_proc = subprocess.Popen(
                        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                    )
                except Exception as exc:
                    self.get_logger().error(f"Lỗi khi gọi {self.player_cmd}: {exc}")
                    self._current_proc = None
                    self._current_key = None
                    self._audio_queue.task_done()
                    continue

            # Đợi phát trọn vẹn câu thoại
            proc = self._current_proc
            if proc:
                try:
                    proc.wait(timeout=25.0)
                except Exception:
                    pass

            with self._lock:
                self._current_proc = None
                self._current_key = None

            self._audio_queue.task_done()

    def cb_play_sound(self, msg: String):
        """Topic /audio/play_sound: nhận trực tiếp tên key hoặc file."""
        text = msg.data.strip()
        if text:
            self.play_sound(text, force=True, priority=False)

    def cb_speech_request(self, msg: String):
        """Topic /navigation/speech_request từ Nav2 Speak & HumanRecoveryBridge."""
        text = msg.data.lower()
        if "lùi" in text or "lui" in text:
            self.play_sound('canh_bao_lui', force=True, priority=True)
        elif "nhường đường" in text or "xin lỗi" in text or "xin phép" in text or "qua" in text:
            self.play_sound('xin_duong', force=False, priority=False)
        elif "tính toán lại" in text or "đổi đường" in text:
            self.play_sound('doi_duong', force=False, priority=False)

    def cb_operator_alert(self, msg: KeyValue):
        """Topic /navigation/operator_alert từ Nav2 Custom Recovery."""
        code = msg.key.strip().upper()
        if code == "PRE_BACKUP":
            self.play_sound('canh_bao_lui', force=True, priority=True)
        elif code == "REPLANNING":
            self.play_sound('doi_duong', force=False, priority=False)
        elif code == "WAITING":
            self.play_sound('xin_duong', force=False, priority=False)

    def cb_mission_log(self, msg: String):
        """Topic /mission/status_log từ Behavior Tree tuần tra & phục vụ."""
        text = msg.data
        if "ĐÃ ĐẾN TRƯỚC MẶT NGƯỜI VẪY TAY" in text:
            # Vừa đến nơi đỗ trước mặt Thầy/Cô vẫy tay -> Chúc mừng 20/11 ngay lập tức
            self.play_sound('chuc_20_11', force=True, priority=False)
        elif "PHÁT HIỆN NGƯỜI VẪY TAY" in text:
            self.play_sound('den_nguoi_vay_tay', force=False, priority=False)
        elif "Hết 5s không yêu cầu nước" in text:
            self.play_sound('tam_biet_5s', force=False, priority=False)
        elif "HOÀN TẤT PHỤC VỤ" in text:
            # Lấy nước xong -> Chỉ cảm ơn ngắn gọn, không phát lặp chúc nữa
            self.play_sound('cam_on_phuc_vu', force=True, priority=False)

    def destroy_node(self):
        self._shutdown_event.set()
        with self._lock:
            if self._current_proc and self._current_proc.poll() is None:
                try:
                    self._current_proc.terminate()
                except Exception:
                    pass
        super().destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
