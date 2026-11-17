"""SLAM and Map Management Page for Robot UI Kiosk.

Provides touch-friendly controls for:
1. SLAM Mapping: Live map display, manual control instructions, start/stop SLAM,
   and dual-format map saving (AMCL YAML + SLAM Toolbox PoseGraph).
2. AMCL Localization: Map directory browsing, interactive map list with static preview,
   backend selection (AMCL vs SLAM Toolbox), and one-touch apply with return to Admin.
"""
from __future__ import annotations

import glob
import os
from pathlib import Path
from typing import Any

import yaml
from PyQt5.QtCore import QPoint, QRect, QRectF, QSize, Qt, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QIcon,
    QImage,
    QImageReader,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
)
from PyQt5.QtWidgets import (
    QButtonGroup,
    QDialog,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.navigation_page import MapWidget


def _find_default_map_dirs() -> list[Path]:
    """Find default locations containing map yaml files across workspace variants."""
    candidates: list[Path] = []
    # 1. Check current working directory
    cwd = Path.cwd().resolve()
    candidates.extend([cwd / "my_map", cwd / "src" / "mo_hinh" / "maps"])

    # 2. Search upwards from current file to find workspace root containing my_map
    curr = Path(__file__).resolve()
    for parent in curr.parents:
        if (parent / "my_map").is_dir():
            candidates.append(parent / "my_map")
        if (parent / "src" / "mo_hinh" / "maps").is_dir():
            candidates.append(parent / "src" / "mo_hinh" / "maps")

    # 3. Check home ros2_ws
    home_ws = Path.home() / "ros2_ws"
    candidates.extend([home_ws / "my_map", home_ws / "src" / "mo_hinh" / "maps"])

    existing: list[Path] = []
    for p in candidates:
        try:
            res = p.resolve()
            if res.is_dir() and res not in existing:
                existing.append(res)
        except Exception:
            pass
    return existing or [cwd / "my_map"]


def check_map_name_exists(map_name: str, map_dirs: list[Path] | Path | None = None) -> bool:
    """Check if a map with this name stem already exists in the given directories.

    Checks for presence of {stem}.yaml, {stem}_amcl.yaml, {stem}_localization.yaml,
    {stem}.pgm, {stem}.posegraph, etc.
    """
    stem = map_name.strip()
    if not stem:
        return False

    dirs: list[Path] = []
    if map_dirs is None:
        dirs = _find_default_map_dirs()
    elif isinstance(map_dirs, Path):
        dirs = [map_dirs]
    else:
        dirs = [p for p in map_dirs if p is not None]

    stems_to_check = {
        stem.lower(),
        f"{stem}_amcl".lower(),
        f"{stem}_localization".lower(),
    }

    if stem.lower().endswith("_amcl"):
        base = stem[:-5]
        if base:
            stems_to_check.add(base.lower())
    elif stem.lower().endswith("_localization"):
        base = stem[:-13]
        if base:
            stems_to_check.add(base.lower())

    for d in dirs:
        if not d.is_dir():
            continue
        try:
            for entry in d.iterdir():
                if not entry.is_file():
                    continue
                stem_lower = entry.stem.lower()
                if stem_lower in stems_to_check:
                    if entry.suffix.lower() in (
                        ".yaml",
                        ".yml",
                        ".pgm",
                        ".png",
                        ".posegraph",
                        ".data",
                    ):
                        return True
        except Exception:
            pass
    return False


class StaticMapPreviewWidget(QWidget):
    """Renders a static preview of a selected map (.yaml -> .pgm/.png)."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("staticMapPreviewWidget")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._map_image: QImage = QImage()
        self._map_name: str = ""
        self._resolution: float = 0.05
        self._image_size: tuple[int, int] = (0, 0)
        self.setMinimumSize(280, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def load_map_yaml(self, yaml_path: Path | str | None) -> bool:
        if not yaml_path:
            self.clear_preview()
            return False

        path = Path(yaml_path)
        if not path.is_file():
            self.clear_preview()
            return False

        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            img_rel = data.get("image", "")
            if not img_rel:
                self.clear_preview()
                return False

            img_path = (path.parent / img_rel).resolve()
            if not img_path.is_file():
                # Try finding in same dir with same stem
                for ext in (".pgm", ".png", ".jpg", ".jpeg"):
                    candidate = path.with_suffix(ext)
                    if candidate.is_file():
                        img_path = candidate
                        break

            if not img_path.is_file():
                self.clear_preview()
                return False

            reader = QImageReader(str(img_path))
            reader.setAutoTransform(True)
            image = reader.read()
            if image.isNull():
                self.clear_preview()
                return False

            # Convert to 32-bit ARGB32_Premultiplied format for safe hardware/software rasterization
            # (raw 8-bit grayscale PGM images can crash QPainter clip path under Wayland/X11).
            image = image.convertToFormat(QImage.Format_ARGB32_Premultiplied)

            self._map_image = image
            self._map_name = path.name
            self._resolution = float(data.get("resolution", 0.05))
            self._image_size = (image.width(), image.height())
            self.update()
            return True
        except Exception:
            self.clear_preview()
            return False

    def clear_preview(self) -> None:
        self._map_image = QImage()
        self._map_name = ""
        self._image_size = (0, 0)
        self.update()

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))

        if self._map_image.isNull():
            painter.setPen(QColor("#64748b"))
            painter.setFont(QFont("Sans Serif", 13, QFont.DemiBold))
            painter.drawText(
                self.rect(),
                Qt.AlignCenter,
                "Chưa có map xem trước\n(Hãy chọn một map bên phải)",
            )
            painter.end()
            return

        w, h = self.width(), self.height()
        pad = 12
        avail_w = max(10, w - pad * 2)
        avail_h = max(10, h - pad * 2 - 40)
        img_w, img_h = self._map_image.width(), self._map_image.height()

        if img_w <= 0 or img_h <= 0 or avail_w <= 0 or avail_h <= 0:
            painter.end()
            return

        # Scale keeping aspect ratio
        scale = min(avail_w / img_w, avail_h / img_h)
        target_w = max(1, int(img_w * scale))
        target_h = max(1, int(img_h * scale))
        target_x = pad + (avail_w - target_w) // 2
        target_y = pad + (avail_h - target_h) // 2
        target_rect = QRect(target_x, target_y, target_w, target_h)

        # Draw frame clip safely
        clip_path = QPainterPath()
        clip_path.addRoundedRect(QRectF(target_rect), 8, 8)
        painter.save()
        painter.setClipPath(clip_path)
        painter.drawImage(target_rect, self._map_image)
        painter.restore()

        # Border
        painter.setPen(QPen(QColor("#242c36"), 2))
        painter.drawRoundedRect(target_rect, 8, 8)

        # Bottom info badge
        info_rect = QRect(pad, h - 38, w - pad * 2, 30)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(17, 20, 24, 230))
        painter.drawRoundedRect(QRectF(info_rect), 6, 6)

        painter.setPen(QColor("#e2e8f0"))
        painter.setFont(QFont("Sans Serif", 10, QFont.DemiBold))
        text = (
            f"Map: {self._map_name}  |  "
            f"Kích thước: {self._image_size[0]}×{self._image_size[1]} px  |  "
            f"Độ phân giải: {self._resolution:.3f} m/cell"
        )
        painter.drawText(info_rect, Qt.AlignCenter, text)
        painter.end()


class TouchMapNameDialog(QDialog):
    """Full-featured on-screen touch keyboard dialog for entering map names without screen clipping."""

    def __init__(
        self,
        current_name: str = "new_map",
        map_dirs: list[Path] | Path | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Đặt Tên Map")
        self.setModal(True)
        self.setFixedSize(680, 460)
        self._map_dirs = map_dirs
        self.setStyleSheet("""
            QDialog {
                background: #ffffff;
                border: 2px solid #dce4ec;
                border-radius: 16px;
            }
            QLabel#touchDialogTitle {
                color: #b32632;
                font-size: 16px;
                font-weight: 800;
            }
            QLineEdit#touchDisplayInput {
                background: #ffffff;
                border: 2px solid #3b82f6;
                border-radius: 10px;
                color: #1d2938;
                font-size: 22px;
                font-weight: 700;
                padding: 0 14px;
            }
            QPushButton#touchKeyBtn {
                background: #f3f6fa;
                border: 1px solid #dce4ec;
                border-radius: 8px;
                color: #1d2938;
                font-size: 16px;
                font-weight: 700;
            }
            QPushButton#touchKeyBtn:hover {
                background: #dce4ec;
            }
            QPushButton#touchKeyBtn:pressed {
                background: #2563eb;
                color: white;
            }
            QPushButton#touchActionKeyBtn {
                background: #dce4ec;
                border: 1px solid #384352;
                border-radius: 8px;
                color: #1d2938;
                font-size: 14px;
                font-weight: 700;
            }
            QPushButton#touchActionKeyBtn:hover {
                background: #d5e0eb;
            }
            QPushButton#touchPresetBtn {
                background: #f3f6fa;
                border: 1px solid #f6c16a;
                border-radius: 6px;
                color: #b32632;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton#touchPresetBtn:hover {
                background: #2a2215;
            }
            QPushButton#touchCancelBtn {
                background: #dce4ec;
                color: #1d2938;
                border: none;
                border-radius: 10px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton#touchConfirmBtn {
                background: #f6c16a;
                color: #0a0a0a;
                border: none;
                border-radius: 10px;
                font-size: 16px;
                font-weight: 800;
            }
            QPushButton#touchConfirmBtn:hover {
                background: #fbbf24;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 14, 20, 14)
        layout.setSpacing(8)

        # Title
        title = QLabel("NHẬP TÊN MAP (BÀN PHÍM CẢM ỨNG)")
        title.setObjectName("touchDialogTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Large Display Field
        self.entry = QLineEdit(current_name)
        self.entry.setObjectName("touchDisplayInput")
        self.entry.setFixedHeight(48)
        self.entry.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.entry)

        # Duplicate Warning / Validation Label
        self.lbl_warning = QLabel("")
        self.lbl_warning.setObjectName("touchDialogWarning")
        self.lbl_warning.setAlignment(Qt.AlignCenter)
        self.lbl_warning.setFixedHeight(20)
        layout.addWidget(self.lbl_warning)

        # Presets Row
        preset_row = QHBoxLayout()
        preset_row.setSpacing(8)
        preset_lbl = QLabel("Gợi ý nhanh:")
        preset_lbl.setStyleSheet("color: #5c6e80; font-size: 12px; font-weight: 600;")
        preset_row.addWidget(preset_lbl)
        for tag in ("tang1", "tang2", "tang3", "lab", "sanh", "kho", "_"):
            btn_tag = QPushButton(tag)
            btn_tag.setObjectName("touchPresetBtn")
            btn_tag.setFixedHeight(30)
            btn_tag.clicked.connect(lambda checked=False, val=tag: self._append_tag(val))
            preset_row.addWidget(btn_tag)
        layout.addLayout(preset_row)

        # Keyboard Grid
        keys_layout = QVBoxLayout()
        keys_layout.setSpacing(6)

        # Row 1: Numbers
        row1 = QHBoxLayout()
        row1.setSpacing(6)
        for num in "1234567890":
            btn = QPushButton(num)
            btn.setObjectName("touchKeyBtn")
            btn.setFixedHeight(44)
            btn.clicked.connect(lambda checked=False, ch=num: self.entry.insert(ch))
            row1.addWidget(btn, 1)
        keys_layout.addLayout(row1)

        # Row 2: QWERTY top
        row2 = QHBoxLayout()
        row2.setSpacing(6)
        for ch in "qwertyuiop":
            btn = QPushButton(ch)
            btn.setObjectName("touchKeyBtn")
            btn.setFixedHeight(44)
            btn.clicked.connect(lambda checked=False, c=ch: self.entry.insert(c))
            row2.addWidget(btn, 1)
        keys_layout.addLayout(row2)

        # Row 3: ASDFGHJKL_
        row3 = QHBoxLayout()
        row3.setSpacing(6)
        for ch in "asdfghjkl_":
            btn = QPushButton(ch)
            btn.setObjectName("touchKeyBtn")
            btn.setFixedHeight(44)
            btn.clicked.connect(lambda checked=False, c=ch: self.entry.insert(c))
            row3.addWidget(btn, 1)
        keys_layout.addLayout(row3)

        # Row 4: ZXCVBNM, Hyphen, Backspace, Clear
        row4 = QHBoxLayout()
        row4.setSpacing(6)
        for ch in "zxcvbnm-":
            btn = QPushButton(ch)
            btn.setObjectName("touchKeyBtn")
            btn.setFixedHeight(44)
            btn.clicked.connect(lambda checked=False, c=ch: self.entry.insert(c))
            row4.addWidget(btn, 1)

        btn_backspace = QPushButton("⌫ Xóa lùi")
        btn_backspace.setObjectName("touchActionKeyBtn")
        btn_backspace.setFixedHeight(44)
        btn_backspace.setMinimumWidth(80)
        btn_backspace.clicked.connect(self.entry.backspace)
        row4.addWidget(btn_backspace, 2)

        btn_clear = QPushButton("Xóa hết")
        btn_clear.setObjectName("touchActionKeyBtn")
        btn_clear.setFixedHeight(44)
        btn_clear.setMinimumWidth(70)
        btn_clear.clicked.connect(self.entry.clear)
        row4.addWidget(btn_clear, 1)

        keys_layout.addLayout(row4)
        layout.addLayout(keys_layout)

        # Action Buttons (Cancel / Confirm)
        actions = QHBoxLayout()
        actions.setSpacing(14)
        btn_cancel = QPushButton("HỦY BỎ")
        btn_cancel.setObjectName("touchCancelBtn")
        btn_cancel.setFixedHeight(46)
        btn_cancel.setCursor(Qt.PointingHandCursor)
        btn_cancel.clicked.connect(self.reject)
        actions.addWidget(btn_cancel, 1)

        self.btn_confirm = QPushButton("XÁC NHẬN (LƯU TÊN)")
        self.btn_confirm.setObjectName("touchConfirmBtn")
        self.btn_confirm.setFixedHeight(46)
        self.btn_confirm.setCursor(Qt.PointingHandCursor)
        self.btn_confirm.clicked.connect(self.accept)
        actions.addWidget(self.btn_confirm, 2)

        layout.addLayout(actions)

        self.entry.textChanged.connect(self._validate_input)
        self._validate_input()

    def _validate_input(self) -> None:
        name = self.entry.text().strip()
        if not name:
            self.entry.setStyleSheet("""
                QLineEdit#touchDisplayInput {
                    background: #1e1b2e;
                    border: 2px solid #f59e0b;
                    border-radius: 10px;
                    color: #fde68a;
                    font-size: 22px;
                    font-weight: 700;
                    padding: 0 14px;
                }
            """)
            self.lbl_warning.setText("⚠️ Hãy nhập tên map")
            self.lbl_warning.setStyleSheet("color: #f59e0b; font-size: 13px; font-weight: 700;")
            self.btn_confirm.setText("XÁC NHẬN (LƯU TÊN)")
            self.btn_confirm.setStyleSheet("""
                QPushButton#touchConfirmBtn {
                    background: #dce4ec;
                    color: #5c6e80;
                    border: none;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: 800;
                }
            """)
            return

        is_dup = check_map_name_exists(name, self._map_dirs)
        if is_dup:
            self.entry.setStyleSheet("""
                QLineEdit#touchDisplayInput {
                    background: #2a1215;
                    border: 2px solid #ef4444;
                    border-radius: 10px;
                    color: #fca5a5;
                    font-size: 22px;
                    font-weight: 700;
                    padding: 0 14px;
                }
            """)
            self.lbl_warning.setText("⚠️ Tên này đã tồn tại trong hệ thống (Lưu sẽ ghi đè!)")
            self.lbl_warning.setStyleSheet("color: #f87171; font-size: 13px; font-weight: 700;")
            self.btn_confirm.setText("XÁC NHẬN (GHI ĐÈ)")
            self.btn_confirm.setStyleSheet("""
                QPushButton#touchConfirmBtn {
                    background: #dc2626;
                    color: #1d2938;
                    border: 2px solid #ef4444;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: 800;
                }
                QPushButton#touchConfirmBtn:hover {
                    background: #b91c1c;
                }
            """)
        else:
            self.entry.setStyleSheet("""
                QLineEdit#touchDisplayInput {
                    background: #0a1c13;
                    border: 2px solid #22c55e;
                    border-radius: 10px;
                    color: #1d2938;
                    font-size: 22px;
                    font-weight: 700;
                    padding: 0 14px;
                }
            """)
            self.lbl_warning.setText("✓ Tên map hợp lệ (chưa tồn tại)")
            self.lbl_warning.setStyleSheet("color: #4ade80; font-size: 13px; font-weight: 600;")
            self.btn_confirm.setText("XÁC NHẬN (LƯU TÊN)")
            self.btn_confirm.setStyleSheet("""
                QPushButton#touchConfirmBtn {
                    background: #f6c16a;
                    color: #0a0a0a;
                    border: none;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: 800;
                }
                QPushButton#touchConfirmBtn:hover {
                    background: #fbbf24;
                }
            """)

    def _append_tag(self, tag: str) -> None:
        cur = self.entry.text().strip()
        if not cur:
            self.entry.setText(tag)
        elif tag == "_":
            self.entry.insert("_")
        else:
            self.entry.setText(f"{cur}_{tag}")

    def get_name(self) -> str:
        return self.entry.text().strip()


class SlamMapPage(QWidget):
    """Kiosk SLAM Mapping and Map Management interface."""

    back_requested = pyqtSignal()
    home_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    fullscreen_toggle_requested = pyqtSignal()
    start_slam_requested = pyqtSignal()
    stop_slam_requested = pyqtSignal()
    save_map_requested = pyqtSignal(str)              # (map_stem)
    apply_map_requested = pyqtSignal(str, str)        # (map_yaml_path, backend)

    def __init__(
        self,
        icons: dict[str, Path | None],
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("slamMapPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.icons = icons

        default_dirs = _find_default_map_dirs()
        self._current_map_dir: Path = default_dirs[0] if default_dirs else Path.cwd()
        self._selected_map_yaml: Path | None = None
        self._is_slam_running = False
        self._is_active = True

        self._init_ui()
        self.refresh_map_files()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 14, 20, 16)
        root.setSpacing(10)

        # 1. Header Bar
        header = QHBoxLayout()
        header.setSpacing(10)

        # Home button
        self.home_button = QPushButton()
        self.home_button.setObjectName("slamHomeButton")
        self.home_button.setCursor(Qt.PointingHandCursor)
        self.home_button.setFixedSize(46, 46)
        self.home_button.setToolTip("Về màn hình chính")
        home_icon = self.icons.get("home")
        if home_icon and Path(home_icon).is_file():
            self.home_button.setIcon(QIcon(str(home_icon)))
            self.home_button.setIconSize(QSize(28, 28))
        else:
            self.home_button.setText("Home")
        self.home_button.clicked.connect(self.home_requested.emit)
        header.addWidget(self.home_button)

        # Emergency Stop button
        self.emergency_button = QPushButton()
        self.emergency_button.setObjectName("slamEmergencyButton")
        self.emergency_button.setCursor(Qt.PointingHandCursor)
        self.emergency_button.setFixedSize(46, 46)
        self.emergency_button.setToolTip("DỪNG KHẨN CẤP")
        emg_icon = self.icons.get("emergency")
        if emg_icon and Path(emg_icon).is_file():
            self.emergency_button.setIcon(QIcon(str(emg_icon)))
            self.emergency_button.setIconSize(QSize(36, 36))
        else:
            self.emergency_button.setText("STOP")
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        header.addWidget(self.emergency_button)

        # Back button
        self.back_button = QPushButton()
        self.back_button.setObjectName("slamBackButton")
        self.back_button.setCursor(Qt.PointingHandCursor)
        self.back_button.setFixedSize(46, 46)
        self.back_button.setToolTip("Quay lại")
        back_icon = self.icons.get("back")
        if back_icon and Path(back_icon).is_file():
            self.back_button.setIcon(QIcon(str(back_icon)))
            self.back_button.setIconSize(QSize(30, 30))
        else:
            self.back_button.setText("‹")
        self.back_button.clicked.connect(self.back_requested.emit)
        header.addWidget(self.back_button)

        # Header Title
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("QUẢN TRỊ MAP & SLAM")
        title.setObjectName("slamTitle")
        subtitle = QLabel("Quét tạo map thời gian thực (SLAM) hoặc nạp map định vị (AMCL)")
        subtitle.setObjectName("slamSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)

        # Status Bar Badges (Wifi & Battery)
        status_bar = QHBoxLayout()
        status_bar.setSpacing(10)

        self.wifi_widget = QWidget()
        self.wifi_widget.setObjectName("slamHeaderBadge")
        wifi_l = QHBoxLayout(self.wifi_widget)
        wifi_l.setContentsMargins(10, 4, 12, 4)
        wifi_l.setSpacing(6)
        self.wifi_icon_label = QLabel()
        self.wifi_icon_label.setFixedSize(26, 26)
        wifi_icon = self.icons.get("wifi_connected")
        if wifi_icon and Path(wifi_icon).is_file():
            self.wifi_icon_label.setPixmap(
                QPixmap(str(wifi_icon)).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        self.wifi_text_label = QLabel("Wifi")
        self.wifi_text_label.setObjectName("slamBadgeText")
        wifi_l.addWidget(self.wifi_icon_label)
        wifi_l.addWidget(self.wifi_text_label)
        status_bar.addWidget(self.wifi_widget)

        self.battery_widget = QWidget()
        self.battery_widget.setObjectName("slamHeaderBadge")
        bat_l = QHBoxLayout(self.battery_widget)
        bat_l.setContentsMargins(10, 4, 12, 4)
        bat_l.setSpacing(6)
        self.battery_icon_label = QLabel()
        self.battery_icon_label.setFixedSize(26, 26)
        bat_icon = (self.icons.get("battery_icons") or {}).get("100")
        if bat_icon and Path(bat_icon).is_file():
            self.battery_icon_label.setPixmap(
                QPixmap(str(bat_icon)).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        self.battery_text_label = QLabel("--%")
        self.battery_text_label.setObjectName("slamBadgeText")
        bat_l.addWidget(self.battery_icon_label)
        bat_l.addWidget(self.battery_text_label)
        status_bar.addWidget(self.battery_widget)

        # Fullscreen Toggle Button
        self.fullscreen_button = QPushButton()
        self.fullscreen_button.setObjectName("slamFullscreenButton")
        self.fullscreen_button.setCursor(Qt.PointingHandCursor)
        self.fullscreen_button.setFixedSize(44, 44)
        self.fullscreen_button.setStyleSheet("""
            QPushButton#slamFullscreenButton {
                background-color: #ffffff;
                border: 1px solid #2a3644;
                border-radius: 8px;
                color: #1d2938;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton#slamFullscreenButton:hover {
                background-color: #f6c16a;
                color: #000000;
                border-color: #f6c16a;
            }
        """)
        self.fullscreen_button.clicked.connect(self.fullscreen_toggle_requested.emit)
        self.update_fullscreen_button(is_fullscreen=True)
        status_bar.addWidget(self.fullscreen_button)

        header.addLayout(status_bar)
        root.addLayout(header)

        # 2. Segmented Mode Switcher (2 Tabs)
        switch_bar = QHBoxLayout()
        switch_bar.setSpacing(12)

        self.btn_tab_slam = QPushButton("QUÉT MAP MỚI (SLAM)")
        self.btn_tab_slam.setObjectName("slamTabButton")
        self.btn_tab_slam.setCursor(Qt.PointingHandCursor)
        self.btn_tab_slam.setCheckable(True)
        self.btn_tab_slam.setChecked(True)
        self.btn_tab_slam.setFixedHeight(48)
        self.btn_tab_slam.clicked.connect(lambda: self._set_active_mode(0))

        self.btn_tab_amcl = QPushButton("ĐỊNH VỊ & CHỌN MAP (AMCL)")
        self.btn_tab_amcl.setObjectName("slamTabButton")
        self.btn_tab_amcl.setCursor(Qt.PointingHandCursor)
        self.btn_tab_amcl.setCheckable(True)
        self.btn_tab_amcl.setChecked(False)
        self.btn_tab_amcl.setFixedHeight(48)
        self.btn_tab_amcl.clicked.connect(lambda: self._set_active_mode(1))

        self.tab_group = QButtonGroup(self)
        self.tab_group.addButton(self.btn_tab_slam, 0)
        self.tab_group.addButton(self.btn_tab_amcl, 1)

        switch_bar.addWidget(self.btn_tab_slam, 1)
        switch_bar.addWidget(self.btn_tab_amcl, 1)
        root.addLayout(switch_bar)

        # 3. Stack of Views
        self.stack = QStackedWidget()
        self.view_slam = self._build_slam_view()
        self.view_amcl = self._build_amcl_view()
        self.stack.addWidget(self.view_slam)
        self.stack.addWidget(self.view_amcl)
        root.addWidget(self.stack, 1)

        self._apply_styles()
        self._set_active_mode(0)
        self._update_name_validation()

    def _build_slam_view(self) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(16)

        # Left Column (60%): Live Map Canvas
        left_box = QFrame()
        left_box.setObjectName("slamMapContainer")
        left_l = QVBoxLayout(left_box)
        left_l.setContentsMargins(8, 8, 8, 8)
        left_l.setSpacing(6)

        live_tag_bar = QHBoxLayout()
        live_tag = QLabel("● MAP SLAM THỜI GIAN THỰC")
        live_tag.setObjectName("slamLiveTag")
        live_tag_bar.addWidget(live_tag)
        live_tag_bar.addStretch()
        left_l.addLayout(live_tag_bar)

        goal_icon = self.icons.get("goal")
        self.live_map_widget = MapWidget(goal_icon=goal_icon)
        self.live_map_widget.set_selection_enabled(False)  # Mapping mode: manual drive, no nav goals
        left_l.addWidget(self.live_map_widget, 1)
        layout.addWidget(left_box, 6)

        # Right Column (40%): Controls & Manual guidance
        right_box = QWidget()
        right_l = QVBoxLayout(right_box)
        right_l.setContentsMargins(0, 0, 0, 0)
        right_l.setSpacing(12)

        # Card 1: Manual Control Warning
        manual_card = QFrame()
        manual_card.setObjectName("slamManualCard")
        manual_l = QHBoxLayout(manual_card)
        manual_l.setContentsMargins(14, 12, 14, 12)
        manual_l.setSpacing(12)

        manual_icon_lbl = QLabel()
        manual_icon_lbl.setFixedSize(64, 64)
        manual_icon_lbl.setAlignment(Qt.AlignCenter)
        m_icon = self.icons.get("manual_control")
        if m_icon and Path(m_icon).is_file():
            manual_icon_lbl.setPixmap(
                QPixmap(str(m_icon)).scaled(60, 60, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        else:
            manual_icon_lbl.setText("MANUAL")
            manual_icon_lbl.setStyleSheet("font-size: 13px; font-weight: 800; color: #f59e0b;")
        manual_l.addWidget(manual_icon_lbl)

        manual_text_box = QVBoxLayout()
        manual_text_box.setSpacing(4)
        m_title = QLabel("ĐIỀU KHIỂN BẰNG TAY (MANUAL)")
        m_title.setObjectName("slamManualTitle")
        m_desc = QLabel(
            "Robot đang trong chế độ tạo map.\n"
            "Hãy dùng tay cầm gamepad hoặc joystick lái robot đi chậm quanh phòng để quét map."
        )
        m_desc.setObjectName("slamManualDesc")
        m_desc.setWordWrap(True)
        manual_text_box.addWidget(m_title)
        manual_text_box.addWidget(m_desc)
        manual_l.addLayout(manual_text_box, 1)
        right_l.addWidget(manual_card)

        # Card 2: SLAM Node Start / Stop
        ctrl_card = QFrame()
        ctrl_card.setObjectName("slamControlCard")
        ctrl_l = QVBoxLayout(ctrl_card)
        ctrl_l.setContentsMargins(14, 12, 14, 12)
        ctrl_l.setSpacing(10)

        self.lbl_slam_status = QLabel("Trạng thái SLAM: Đang dừng")
        self.lbl_slam_status.setObjectName("slamStatusLabel")
        ctrl_l.addWidget(self.lbl_slam_status)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        self.btn_start_slam = QPushButton("▶  BẮT ĐẦU QUÉT")
        self.btn_start_slam.setObjectName("slamStartButton")
        self.btn_start_slam.setCursor(Qt.PointingHandCursor)
        self.btn_start_slam.setFixedHeight(46)
        self.btn_start_slam.clicked.connect(self._on_start_slam_clicked)

        self.btn_stop_slam = QPushButton("■  DỪNG QUÉT")
        self.btn_stop_slam.setObjectName("slamStopButton")
        self.btn_stop_slam.setCursor(Qt.PointingHandCursor)
        self.btn_stop_slam.setFixedHeight(46)
        self.btn_stop_slam.clicked.connect(self._on_stop_slam_clicked)

        btn_row.addWidget(self.btn_start_slam, 1)
        btn_row.addWidget(self.btn_stop_slam, 1)
        ctrl_l.addLayout(btn_row)
        right_l.addWidget(ctrl_card)

        # Card 3: Save Map Panel
        save_card = QFrame()
        save_card.setObjectName("slamSaveCard")
        save_l = QVBoxLayout(save_card)
        save_l.setContentsMargins(14, 12, 14, 12)
        save_l.setSpacing(8)

        save_title = QLabel("● LƯU MAP HIỆN TẠI:")
        save_title.setObjectName("slamSaveTitle")
        save_l.addWidget(save_title)

        name_row = QHBoxLayout()
        name_row.setSpacing(8)

        self.edt_map_name = QLineEdit("new_map")
        self.edt_map_name.setObjectName("slamMapNameInput")
        self.edt_map_name.setFixedHeight(44)
        self.edt_map_name.setPlaceholderText("Nhập tên map...")
        self.edt_map_name.setReadOnly(True)  # Touch-first: tap opens full on-screen keyboard
        self.edt_map_name.mousePressEvent = lambda e: self._open_touch_keyboard()

        self.btn_open_keyboard = QPushButton("Bàn Phím")
        self.btn_open_keyboard.setObjectName("slamActionMiniButton")
        self.btn_open_keyboard.setFixedHeight(44)
        self.btn_open_keyboard.setFixedWidth(90)
        self.btn_open_keyboard.setCursor(Qt.PointingHandCursor)
        self.btn_open_keyboard.clicked.connect(self._open_touch_keyboard)

        name_row.addWidget(self.edt_map_name, 1)
        name_row.addWidget(self.btn_open_keyboard)
        save_l.addLayout(name_row)

        # Name duplicate warning / validation label
        self.lbl_name_warning = QLabel("")
        self.lbl_name_warning.setObjectName("slamNameWarningLabel")
        self.lbl_name_warning.setWordWrap(True)
        save_l.addWidget(self.lbl_name_warning)

        # Quick preset buttons for touch operation without keyboard
        quick_row = QHBoxLayout()
        quick_row.setSpacing(6)
        presets = ["lab", "tang1", "tang2", "sanh"]
        for p in presets:
            btn_p = QPushButton(p)
            btn_p.setObjectName("slamPresetButton")
            btn_p.setFixedHeight(32)
            btn_p.clicked.connect(lambda checked=False, val=p: self.edt_map_name.setText(val))
            quick_row.addWidget(btn_p)
        btn_clear = QPushButton("Xóa")
        btn_clear.setObjectName("slamPresetButton")
        btn_clear.setFixedHeight(32)
        btn_clear.clicked.connect(self.edt_map_name.clear)
        quick_row.addWidget(btn_clear)
        save_l.addLayout(quick_row)

        self.btn_save_map = QPushButton("LƯU MAP (DUAL-SAVE)")
        self.btn_save_map.setObjectName("slamSaveButton")
        self.btn_save_map.setCursor(Qt.PointingHandCursor)
        self.btn_save_map.setFixedHeight(50)
        self.btn_save_map.clicked.connect(self._on_save_map_clicked)
        save_l.addWidget(self.btn_save_map)

        self.lbl_save_status = QLabel("Sẵn sàng lưu map khi quét hoàn tất")
        self.lbl_save_status.setObjectName("slamSaveStatus")
        self.lbl_save_status.setWordWrap(True)
        save_l.addWidget(self.lbl_save_status)

        self.edt_map_name.textChanged.connect(self._update_name_validation)

        right_l.addWidget(save_card)
        right_l.addStretch(1)
        layout.addWidget(right_box, 4)

        return container

    def _build_amcl_view(self) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(16)

        # Left Column (55%): Static Map Preview
        left_box = QFrame()
        left_box.setObjectName("slamMapContainer")
        left_l = QVBoxLayout(left_box)
        left_l.setContentsMargins(8, 8, 8, 8)
        left_l.setSpacing(6)

        prev_tag_bar = QHBoxLayout()
        prev_tag = QLabel("● XEM TRƯỚC MAP TĨNH")
        prev_tag.setObjectName("slamLiveTag")
        prev_tag_bar.addWidget(prev_tag)
        prev_tag_bar.addStretch()
        left_l.addLayout(prev_tag_bar)

        self.map_preview_widget = StaticMapPreviewWidget()
        left_l.addWidget(self.map_preview_widget, 1)
        layout.addWidget(left_box, 5)

        # Right Column (45%): Map selection & Backend options
        right_box = QWidget()
        right_l = QVBoxLayout(right_box)
        right_l.setContentsMargins(0, 0, 0, 0)
        right_l.setSpacing(10)

        # Folder row
        folder_card = QFrame()
        folder_card.setObjectName("slamControlCard")
        folder_l = QVBoxLayout(folder_card)
        folder_l.setContentsMargins(12, 10, 12, 10)
        folder_l.setSpacing(6)

        self.lbl_folder_path = QLabel(f"Thư mục: {self._current_map_dir}")
        self.lbl_folder_path.setObjectName("slamFolderPathLabel")
        self.lbl_folder_path.setWordWrap(True)
        folder_l.addWidget(self.lbl_folder_path)

        dir_btn_row = QHBoxLayout()
        dir_btn_row.setSpacing(8)
        self.btn_choose_dir = QPushButton("Chọn Thư Mục")
        self.btn_choose_dir.setObjectName("slamActionMiniButton")
        self.btn_choose_dir.setFixedHeight(38)
        self.btn_choose_dir.clicked.connect(self._on_choose_folder_clicked)

        self.btn_refresh_maps = QPushButton("Làm Mới")
        self.btn_refresh_maps.setObjectName("slamActionMiniButton")
        self.btn_refresh_maps.setFixedHeight(38)
        self.btn_refresh_maps.clicked.connect(self.refresh_map_files)

        dir_btn_row.addWidget(self.btn_choose_dir, 1)
        dir_btn_row.addWidget(self.btn_refresh_maps, 1)
        folder_l.addLayout(dir_btn_row)
        right_l.addWidget(folder_card)

        # Map list
        list_card = QFrame()
        list_card.setObjectName("slamControlCard")
        list_l = QVBoxLayout(list_card)
        list_l.setContentsMargins(12, 10, 12, 10)
        list_l.setSpacing(6)

        list_title = QLabel("CHỌN MAP VẬN HÀNH:")
        list_title.setObjectName("slamSaveTitle")
        list_l.addWidget(list_title)

        self.map_list_widget = QListWidget()
        self.map_list_widget.setObjectName("slamMapList")
        self.map_list_widget.itemSelectionChanged.connect(self._on_map_selection_changed)
        list_l.addWidget(self.map_list_widget, 1)

        # Backend radio selection
        backend_row = QHBoxLayout()
        backend_row.setSpacing(14)
        backend_lbl = QLabel("Thuật toán:")
        backend_lbl.setStyleSheet("color: #5c6e80; font-size: 13px; font-weight: 700;")
        backend_row.addWidget(backend_lbl)

        self.rb_amcl = QRadioButton("AMCL (Khuyên dùng)")
        self.rb_amcl.setObjectName("slamRadio")
        self.rb_amcl.setChecked(True)
        self.rb_slam_loc = QRadioButton("SLAM Toolbox Graph")
        self.rb_slam_loc.setObjectName("slamRadio")
        backend_row.addWidget(self.rb_amcl)
        backend_row.addWidget(self.rb_slam_loc)
        backend_row.addStretch()
        list_l.addLayout(backend_row)
        right_l.addWidget(list_card, 1)

        # Big Apply & Return CTA
        self.btn_apply_map = QPushButton("ÁP DỤNG MAP & VỀ ADMIN")
        self.btn_apply_map.setObjectName("slamApplyButton")
        self.btn_apply_map.setCursor(Qt.PointingHandCursor)
        self.btn_apply_map.setFixedHeight(54)
        self.btn_apply_map.clicked.connect(self._on_apply_map_clicked)
        right_l.addWidget(self.btn_apply_map)

        layout.addWidget(right_box, 5)
        return container

    def _open_touch_keyboard(self) -> None:
        dirs = [self._current_map_dir] + _find_default_map_dirs()
        dialog = TouchMapNameDialog(self.edt_map_name.text().strip(), dirs, self)
        if dialog.exec_() == QDialog.Accepted:
            new_name = dialog.get_name()
            if new_name:
                self.edt_map_name.setText(new_name)
                self._update_name_validation()

    def _set_active_mode(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        self.btn_tab_slam.setChecked(index == 0)
        self.btn_tab_amcl.setChecked(index == 1)

    def _on_choose_folder_clicked(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self,
            "Chọn thư mục chứa map YAML",
            str(self._current_map_dir) if self._current_map_dir.exists() else str(Path.home()),
        )
        if folder:
            self._current_map_dir = Path(folder)
            self.lbl_folder_path.setText(f"Thư mục: {self._current_map_dir}")
            self.refresh_map_files()

    def refresh_map_files(self) -> None:
        self.map_list_widget.clear()
        if not self._current_map_dir.exists():
            return

        all_yamls = sorted(self._current_map_dir.glob("*.yaml")) + sorted(self._current_map_dir.glob("*.yml"))
        # Filter: only include genuine Nav2 map yamls (containing 'image:' or 'resolution:')
        valid_yamls: list[Path] = []
        for y in all_yamls:
            try:
                with y.open("r", encoding="utf-8") as f:
                    snippet = f.read(512)
                    if "image:" in snippet or "resolution:" in snippet:
                        valid_yamls.append(y)
            except Exception:
                pass

        for y in valid_yamls:
            item = QListWidgetItem(f"[Map]  {y.name}")
            item.setData(Qt.UserRole, str(y))
            self.map_list_widget.addItem(item)

        if valid_yamls:
            self.map_list_widget.setCurrentRow(0)
            self._on_map_selection_changed()
        else:
            self.map_preview_widget.clear_preview()
            self._selected_map_yaml = None
        self._update_name_validation()

    def _update_name_validation(self) -> None:
        """Validate currently entered map name and show real-time warning if already exists."""
        if not hasattr(self, "edt_map_name") or not hasattr(self, "lbl_name_warning"):
            return

        name = self.edt_map_name.text().strip()
        if not name:
            self.lbl_name_warning.setText("⚠️ Hãy nhập tên map")
            self.lbl_name_warning.setStyleSheet("color: #f59e0b; font-size: 12px; font-weight: 600;")
            self.edt_map_name.setStyleSheet("""
                QLineEdit#slamMapNameInput {
                    background: #f3f6fa;
                    border: 2px solid #f59e0b;
                    border-radius: 8px;
                    color: white;
                    font-size: 15px;
                    font-weight: 600;
                    padding: 0 12px;
                }
            """)
            self.btn_save_map.setText("LƯU MAP (DUAL-SAVE)")
            self.btn_save_map.setStyleSheet("")
            return

        dirs = [self._current_map_dir] + _find_default_map_dirs()
        is_dup = check_map_name_exists(name, dirs)
        if is_dup:
            self.lbl_name_warning.setText("⚠️ Tên map đã tồn tại (Lưu sẽ ghi đè lên map cũ!)")
            self.lbl_name_warning.setStyleSheet("color: #f87171; font-size: 12px; font-weight: 700;")
            self.edt_map_name.setStyleSheet("""
                QLineEdit#slamMapNameInput {
                    background: #2a1215;
                    border: 2px solid #ef4444;
                    border-radius: 8px;
                    color: #fca5a5;
                    font-size: 15px;
                    font-weight: 700;
                    padding: 0 12px;
                }
            """)
            self.btn_save_map.setText("⚠️ LƯU GHI ĐÈ MAP (OVERWRITE)")
            self.btn_save_map.setStyleSheet("""
                QPushButton#slamSaveButton {
                    background: #dc2626;
                    color: white;
                    border: 2px solid #ef4444;
                    border-radius: 10px;
                    font-size: 15px;
                    font-weight: 700;
                }
                QPushButton#slamSaveButton:hover {
                    background: #b91c1c;
                }
            """)
        else:
            self.lbl_name_warning.setText("✓ Tên map hợp lệ (chưa tồn tại)")
            self.lbl_name_warning.setStyleSheet("color: #4ade80; font-size: 12px; font-weight: 600;")
            self.edt_map_name.setStyleSheet("""
                QLineEdit#slamMapNameInput {
                    background: #0a1c13;
                    border: 1.5px solid #10b981;
                    border-radius: 8px;
                    color: #1d2938;
                    font-size: 15px;
                    font-weight: 600;
                    padding: 0 12px;
                }
            """)
            self.btn_save_map.setText("LƯU MAP (DUAL-SAVE)")
            self.btn_save_map.setStyleSheet("")

    def _on_map_selection_changed(self) -> None:
        selected_items = self.map_list_widget.selectedItems()
        if not selected_items:
            self.map_preview_widget.clear_preview()
            self._selected_map_yaml = None
            return

        path_str = selected_items[0].data(Qt.UserRole)
        self._selected_map_yaml = Path(path_str)
        self.map_preview_widget.load_map_yaml(self._selected_map_yaml)

    def _on_start_slam_clicked(self) -> None:
        self._is_slam_running = True
        self.lbl_slam_status.setText("Trạng thái SLAM: ● ĐANG QUÉT MAP")
        self.lbl_slam_status.setStyleSheet("color: #22c55e; font-weight: bold;")
        self.btn_start_slam.setEnabled(False)
        self.btn_stop_slam.setEnabled(True)
        self.lbl_save_status.setText("Đang ghi nhận dữ liệu từ cảm biến LiDAR...")
        self.start_slam_requested.emit()

    def _on_stop_slam_clicked(self) -> None:
        self._is_slam_running = False
        self.lbl_slam_status.setText("Trạng thái SLAM: Đã dừng quét")
        self.lbl_slam_status.setStyleSheet("color: #f59e0b; font-weight: bold;")
        self.btn_start_slam.setEnabled(True)
        self.btn_stop_slam.setEnabled(False)
        self.stop_slam_requested.emit()

    def _on_save_map_clicked(self) -> None:
        map_name = self.edt_map_name.text().strip()
        if not map_name:
            self.lbl_save_status.setText("⚠️ Hãy nhập tên map trước khi lưu")
            self.lbl_save_status.setStyleSheet("color: #ef4444; font-weight: bold;")
            return

        dirs = [self._current_map_dir] + _find_default_map_dirs()
        is_dup = check_map_name_exists(map_name, dirs)
        if is_dup:
            self.lbl_save_status.setText(f"⏳ Đang ghi đè map '{map_name}' (AMCL + PoseGraph)...")
            self.lbl_save_status.setStyleSheet("color: #fb923c; font-weight: bold;")
        else:
            self.lbl_save_status.setText(f"⏳ Đang lưu map mới '{map_name}' (AMCL + PoseGraph)...")
            self.lbl_save_status.setStyleSheet("color: #126da8; font-weight: bold;")
        self.save_map_requested.emit(map_name)

    def _on_apply_map_clicked(self) -> None:
        if not self._selected_map_yaml or not self._selected_map_yaml.is_file():
            self.lbl_save_status.setText("⚠️ Vui lòng chọn một map YAML hợp lệ")
            return

        backend = "amcl" if self.rb_amcl.isChecked() else "slam_toolbox"
        self.apply_map_requested.emit(str(self._selected_map_yaml), backend)

    def set_wifi(self, connected: bool) -> None:
        if connected:
            self.wifi_text_label.setText("Online")
            self.wifi_text_label.setStyleSheet("color: #4ade80;")
            icon = self.icons.get("wifi_connected")
        else:
            self.wifi_text_label.setText("Offline")
            self.wifi_text_label.setStyleSheet("color: #f87171;")
            icon = self.icons.get("wifi_disconnected")
        if icon and Path(icon).is_file():
            pm = QPixmap(str(icon))
            if not pm.isNull():
                self.wifi_icon_label.setPixmap(
                    pm.scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )

    def set_battery(self, percent: float | None) -> None:
        if percent is None:
            self.battery_text_label.setText("--%")
            return
        try:
            p = float(percent)
        except (ValueError, TypeError):
            self.battery_text_label.setText("--%")
            return
        p = max(0.0, min(100.0, p))
        self.battery_text_label.setText(f"{round(p)}%")
        bucket = "100" if p >= 85 else ("75" if p >= 60 else ("50" if p >= 35 else ("25" if p >= 15 else "0")))
        bat_icon = (self.icons.get("battery_icons") or {}).get(bucket)
        if bat_icon and Path(bat_icon).is_file():
            pm = QPixmap(str(bat_icon))
            if not pm.isNull():
                self.battery_icon_label.setPixmap(
                    pm.scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        self.battery_text_label.setText(f"{round(p)}%")
        bucket = "100" if p >= 85 else ("75" if p >= 60 else ("50" if p >= 35 else ("25" if p >= 15 else "0")))
        bat_icon = (self.icons.get("battery_icons") or {}).get(bucket)
        if bat_icon and Path(bat_icon).is_file():
            self.battery_icon_label.setPixmap(
                QPixmap(str(bat_icon)).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    def update_fullscreen_button(self, is_fullscreen: bool) -> None:
        """Update single button icon and tooltip alternating between fullscreen and windowed mode."""
        if is_fullscreen:
            self.fullscreen_button.setText("🗗")
            self.fullscreen_button.setToolTip("Thoát toàn màn hình (Esc hoặc F11)")
        else:
            self.fullscreen_button.setText("🗖")
            self.fullscreen_button.setToolTip("Toàn màn hình (F11)")

    def set_active(self, active: bool) -> None:
        """Kích hoạt hoặc tắt render nền cho widget live map của SLAM."""
        self._is_active = active
        if hasattr(self, "live_map_widget") and self.live_map_widget is not None:
            self.live_map_widget.set_active(active)

    def set_map(self, map_data: dict[str, Any]) -> None:
        """Cập nhật dữ liệu bản đồ thời gian thực lên live map widget."""
        if hasattr(self, "live_map_widget") and self.live_map_widget is not None:
            self.live_map_widget.set_map(map_data)

    def set_pose(self, pose: dict[str, Any]) -> None:
        """Cập nhật vị trí robot lên live map widget."""
        if hasattr(self, "live_map_widget") and self.live_map_widget is not None:
            self.live_map_widget.set_pose(pose)

    def set_scan_points(self, points: list[tuple[float, float]]) -> None:
        """Cập nhật các điểm laser LiDAR lên live map widget."""
        if hasattr(self, "live_map_widget") and self.live_map_widget is not None:
            self.live_map_widget.set_scan_points(points)

    def set_obstacle_points(self, side: str, points: list[tuple[float, float]]) -> None:
        """Cập nhật các điểm chướng ngại vật từ camera độ sâu lên live map widget."""
        if hasattr(self, "live_map_widget") and self.live_map_widget is not None:
            self.live_map_widget.set_obstacle_points(side, points)

    def set_slam_status(self, is_running: bool, message: str = "") -> None:
        self._is_slam_running = is_running
        self.btn_start_slam.setEnabled(not is_running)
        self.btn_stop_slam.setEnabled(is_running)
        if is_running:
            self.lbl_slam_status.setText("Trạng thái SLAM: ● ĐANG QUÉT MAP")
            self.lbl_slam_status.setStyleSheet("color: #22c55e; font-weight: bold;")
        else:
            self.lbl_slam_status.setText("Trạng thái SLAM: Đang dừng")
            self.lbl_slam_status.setStyleSheet("color: #5c6e80; font-weight: bold;")
        if message:
            self.lbl_save_status.setText(message)

    def set_save_result(self, success: bool, message: str) -> None:
        if success:
            self.lbl_save_status.setText(f"✅ {message}")
            self.lbl_save_status.setStyleSheet("color: #22c55e; font-weight: bold;")
            self.refresh_map_files()
        else:
            self.lbl_save_status.setText(f"❌ {message}")
            self.lbl_save_status.setStyleSheet("color: #ef4444; font-weight: bold;")

    def _apply_styles(self) -> None:
        self.setStyleSheet("""
            QWidget#slamMapPage {
                background: #f7f9fc;
            }
            QLabel {
                color: #263747;
            }
            QLabel#slamTitle {
                color: #b32632;
                font-size: 24px;
                font-weight: 800;
            }
            QLabel#slamSubtitle {
                color: #5c6e80;
                font-size: 13px;
                font-weight: 500;
            }
            QWidget#slamHeaderBadge {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-radius: 10px;
            }
            QLabel#slamBadgeText {
                color: #1d2938;
                font-size: 13px;
                font-weight: 700;
            }
            QPushButton#slamBackButton, QPushButton#slamHomeButton, QPushButton#slamEmergencyButton {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-radius: 12px;
                color: white;
            }
            QPushButton#slamBackButton:hover, QPushButton#slamHomeButton:hover {
                background: #1c222b;
                border-color: #384352;
            }
            QPushButton#slamEmergencyButton:pressed {
                background: #7f1d1d;
            }
            QPushButton#slamTabButton {
                background: #ffffff;
                border: 2px solid #dce4ec;
                border-radius: 12px;
                color: #5c6e80;
                font-size: 15px;
                font-weight: 700;
                padding: 0 16px;
            }
            QPushButton#slamTabButton:checked {
                background: #1c222b;
                border-color: #f6c16a;
                color: #b32632;
            }
            QFrame#slamMapContainer {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-radius: 14px;
            }
            QLabel#slamLiveTag {
                color: #126da8;
                font-size: 12px;
                font-weight: 800;
                padding: 2px 6px;
            }
            QFrame#slamManualCard {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-left: 4px solid #f6c16a;
                border-radius: 12px;
            }
            QLabel#slamManualTitle {
                color: #b32632;
                font-size: 14px;
                font-weight: 800;
            }
            QLabel#slamManualDesc {
                color: #506176;
                font-size: 12px;
                line-height: 1.4;
            }
            QFrame#slamControlCard, QFrame#slamSaveCard {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-radius: 12px;
            }
            QLabel#slamStatusLabel {
                color: #5c6e80;
                font-size: 13px;
                font-weight: 700;
            }
            QPushButton#slamStartButton {
                background: #16a34a;
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton#slamStartButton:hover {
                background: #15803d;
            }
            QPushButton#slamStartButton:disabled {
                background: #14532d;
                color: #86efac;
            }
            QPushButton#slamStopButton {
                background: #dc2626;
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton#slamStopButton:hover {
                background: #b91c1c;
            }
            QPushButton#slamStopButton:disabled {
                background: #450a0a;
                color: #fca5a5;
            }
            QLabel#slamSaveTitle {
                color: #1d2938;
                font-size: 14px;
                font-weight: 800;
            }
            QLineEdit#slamMapNameInput {
                background: #f3f6fa;
                border: 1px solid #dce4ec;
                border-radius: 8px;
                color: white;
                font-size: 15px;
                font-weight: 600;
                padding: 0 12px;
            }
            QLineEdit#slamMapNameInput:focus {
                border-color: #f6c16a;
            }
            QPushButton#slamPresetButton {
                background: #f3f6fa;
                border: 1px solid #dce4ec;
                border-radius: 6px;
                color: #506176;
                font-size: 12px;
                font-weight: 600;
            }
            QPushButton#slamPresetButton:hover {
                background: #dce4ec;
            }
            QPushButton#slamSaveButton {
                background: #2563eb;
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton#slamSaveButton:hover {
                background: #1d4ed8;
            }
            QLabel#slamSaveStatus {
                color: #5c6e80;
                font-size: 12px;
                min-height: 20px;
            }
            QLabel#slamFolderPathLabel {
                color: #5c6e80;
                font-size: 12px;
            }
            QPushButton#slamActionMiniButton {
                background: #f3f6fa;
                border: 1px solid #dce4ec;
                border-radius: 8px;
                color: #1d2938;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton#slamActionMiniButton:hover {
                background: #dce4ec;
            }
            QListWidget#slamMapList {
                background: #ffffff;
                border: 1px solid #dce4ec;
                border-radius: 8px;
                color: #1d2938;
                font-size: 14px;
                font-weight: 600;
                padding: 4px;
            }
            QListWidget#slamMapList::item {
                height: 42px;
                padding-left: 8px;
                border-radius: 6px;
            }
            QListWidget#slamMapList::item:hover {
                background: #f3f6fa;
            }
            QListWidget#slamMapList::item:selected {
                background: #2a2215;
                color: #b32632;
                border: 1px solid #f6c16a;
            }
            QRadioButton#slamRadio {
                color: #506176;
                font-size: 13px;
                font-weight: 600;
            }
            QRadioButton#slamRadio::indicator {
                width: 16px;
                height: 16px;
            }
            QPushButton#slamApplyButton {
                background: #f6c16a;
                color: #000000;
                border: none;
                border-radius: 12px;
                font-size: 16px;
                font-weight: 800;
            }
            QPushButton#slamApplyButton:hover {
                background: #fbbf24;
            }
            QPushButton#slamApplyButton:pressed {
                background: #d97706;
            }
        """)
