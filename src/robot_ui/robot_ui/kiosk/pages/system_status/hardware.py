"""hardware.py - Screen 2: Module B (Hardware & Jetson Detail).

Organizes:
1. Power & Service: Deep 36V battery telemetry (Moving Average curve) & Water/Kettle status.
2. Jetson Orin Nano Telemetry: CPU, GPU Tegra, RAM, SSD, SoC Temperature with >=75°C alarm.
3. Connectivity: Local IP, Wi-Fi SSID & Signal Strength %, 9Router Gateway & Voice STT.
"""
from __future__ import annotations

from typing import Any, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QScroller,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.components import (
    MetricCard,
    PageHeader,
    create_section_card,
)
from robot_ui.kiosk.pages.system_status.contracts import (
    HealthStatus,
    HostTelemetrySnapshot,
    PowerWaterSnapshot,
)
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


class HardwareDetailScreen(QWidget):
    """Screen 2: Hardware & Host Telemetry Detail Screen."""

    back_requested = pyqtSignal()

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("hardwareDetailScreen")
        self.icons = icons or {}
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 12, 18, 16)
        layout.setSpacing(10)

        # 1. Header with Back button
        self.header = PageHeader(
            title="PHẦN CỨNG & NGUỒN ĐIỆN",
            subtitle="Giám sát Tài nguyên Máy chủ Jetson, Ắc quy 36V & Kết nối Mạng",
            back_callback=self.back_requested.emit,
            back_icon_path=self.icons.get("back"),
            status_text="ĐANG KIỂM TRA...",
            status=HealthStatus.UNKNOWN,
        )
        layout.addLayout(self.header)

        # 2. Scroll Area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background: transparent; border: none;")
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)

        container = QWidget()
        c_layout = QVBoxLayout(container)
        c_layout.setContentsMargins(0, 0, 4, 0)
        c_layout.setSpacing(12)

        # A. NGUỒN ĐIỆN ẮC QUY 36V & NƯỚC
        card_power, p_layout = create_section_card("NGUỒN ĐIỆN ẮC QUY 36V & BÌNH PHỤC VỤ NƯỚC", "#22C55E")
        p_row = QHBoxLayout()
        p_row.setSpacing(14)

        self.lbl_bat_pct = QLabel("--%")
        self.lbl_bat_pct.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 28px; font-weight: 900; min-width: 80px;")
        p_row.addWidget(self.lbl_bat_pct)

        bat_detail_col = QVBoxLayout()
        bat_detail_col.setSpacing(4)
        self.pbar_bat = QProgressBar()
        self.pbar_bat.setRange(0, 100)
        self.pbar_bat.setValue(0)
        self.pbar_bat.setFixedHeight(16)
        self.pbar_bat.setTextVisible(False)
        self.pbar_bat.setStyleSheet(f"""
            QProgressBar {{ border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 4px; background: {KioskTheme.COLOR_SURFACE}; }}
            QProgressBar::chunk {{ background: {KioskTheme.COLOR_HEALTHY}; border-radius: 3px; }}
        """)
        bat_detail_col.addWidget(self.pbar_bat)

        self.lbl_bat_sub = QLabel("Điện áp: -- V | Nguồn: --")
        self.lbl_bat_sub.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")
        bat_detail_col.addWidget(self.lbl_bat_sub)
        p_row.addLayout(bat_detail_col, stretch=1)
        p_layout.addLayout(p_row)

        # Sub-row hiển thị thông số Bình Nước & Khay Ấm
        water_row = QHBoxLayout()
        water_row.setSpacing(14)
        self.lbl_water_val = QLabel("-- L")
        self.lbl_water_val.setStyleSheet(f"color: {KioskTheme.COLOR_ACCENT}; font-size: 18px; font-weight: 800; min-width: 80px;")
        water_row.addWidget(self.lbl_water_val)

        w_col = QVBoxLayout()
        w_col.setSpacing(2)
        self.lbl_kettle_status = QLabel("Khay ấm: Đang kiểm tra...")
        self.lbl_kettle_status.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 11px; font-weight: bold;")
        self.lbl_water_sub = QLabel("Mực nước bình chứa: --")
        self.lbl_water_sub.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 10px;")
        w_col.addWidget(self.lbl_kettle_status)
        w_col.addWidget(self.lbl_water_sub)
        water_row.addLayout(w_col, stretch=1)
        p_layout.addLayout(water_row)

        c_layout.addWidget(card_power)

        # B. PROCESS MONITOR (GIÁM SÁT TIẾN TRÌNH CPU & RAM)
        card_proc, proc_layout = create_section_card("GIÁM SÁT TIẾN TRÌNH HỆ THỐNG (PROCESS MONITOR)", "#38BDF8")

        proc_top = QHBoxLayout()
        lbl_proc_desc = QLabel("Xem danh sách các tiến trình chiếm dụng tài nguyên CPU / RAM cao nhất")
        lbl_proc_desc.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")
        proc_top.addWidget(lbl_proc_desc)
        proc_top.addStretch()

        self.btn_toggle_proc = QPushButton("Hiện Tiến Trình (Process Monitor)")
        self.btn_toggle_proc.setFixedHeight(34)
        self.btn_toggle_proc.setCursor(Qt.PointingHandCursor)
        self.btn_toggle_proc.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: {KioskTheme.COLOR_ACCENT};
                font-size: 11px;
                font-weight: bold;
                padding: 0 12px;
            }}
            QPushButton:hover {{
                background-color: {KioskTheme.COLOR_SURFACE_ACTIVE};
                border-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)
        self.btn_toggle_proc.clicked.connect(self._toggle_proc_table)
        proc_top.addWidget(self.btn_toggle_proc)
        proc_layout.addLayout(proc_top)
        self._last_top_processes: list[dict[str, Any]] = []

        # Table of processes
        self.table_proc = QTableWidget()
        self.table_proc.setColumnCount(5)
        self.table_proc.setHorizontalHeaderLabels(["PID", "Tiến Trình (Process)", "% CPU", "RAM (MB)", "Trạng Thái"])
        self.table_proc.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_proc.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_proc.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_proc.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_proc.setMinimumHeight(160)
        self.table_proc.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table_proc.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_proc.setStyleSheet(f"""
            QTableWidget {{
                background-color: #0b0f15;
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                gridline-color: #1e2633;
                font-family: monospace;
                font-size: 11px;
            }}
            QHeaderView::section {{
                background-color: {KioskTheme.COLOR_SURFACE};
                color: {KioskTheme.COLOR_TEXT_SECONDARY};
                font-weight: bold;
                font-size: 11px;
                border: none;
                border-bottom: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                padding: 4px;
            }}
        """)
        self.table_proc.hide()
        proc_layout.addWidget(self.table_proc)
        c_layout.addWidget(card_proc)

        # B. TÀI NGUYÊN MÁY CHỦ JETSON ORIN NANO
        card_jetson, j_layout = create_section_card("HIỆU NĂNG PHẦN CỨNG JETSON ORIN NANO", KioskTheme.COLOR_ACCENT)
        j_grid = QGridLayout()
        j_grid.setSpacing(10)

        self.card_cpu = MetricCard("Tải Vi Xử Lý CPU", "0.0%", "Nhiệt độ: --°C")
        self.card_gpu = MetricCard("Nhân Đồ Họa GPU Tegra", "0.0%", "Nhiệt độ: --°C")
        self.card_ram = MetricCard("Bộ Nhớ RAM", "0.0 / 0.0 GB", "0% đã dùng")
        self.card_disk = MetricCard("Ổ Cứng SSD", "0.0 GB trống", "0% đã dùng")

        j_grid.addWidget(self.card_cpu, 0, 0)
        j_grid.addWidget(self.card_gpu, 0, 1)
        j_grid.addWidget(self.card_ram, 1, 0)
        j_grid.addWidget(self.card_disk, 1, 1)
        j_layout.addLayout(j_grid)

        # Thermal Warning Notice
        self.lbl_thermal_notice = QLabel("Nhiệt độ SoC an toàn (<75°C).")
        self.lbl_thermal_notice.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 11px; font-weight: bold; padding: 4px 0;")
        j_layout.addWidget(self.lbl_thermal_notice)
        c_layout.addWidget(card_jetson)

        # C. KẾT NỐI MẠNG & DỊCH VỤ AI
        card_net, n_layout = create_section_card("KẾT NỐI MẠNG & DỊCH VỤ AI", "#38BDF8")
        n_grid = QGridLayout()
        n_grid.setSpacing(10)

        self.card_ip = MetricCard("Địa chỉ IP", "127.0.0.1", "LAN / Wi-Fi")
        self.card_wifi = MetricCard("Mạng Wi-Fi", "Connected", "Sóng: 0%")
        self.card_router = MetricCard("9Router Gateway", "OFFLINE", "Port 20128 · LLM & TTS")
        self.card_stt = MetricCard("Voice STT", "OFFLINE", "Groq Whisper")
        self.card_cloud = self.card_stt

        n_grid.addWidget(self.card_ip, 0, 0)
        n_grid.addWidget(self.card_wifi, 0, 1)
        n_grid.addWidget(self.card_router, 0, 2)
        n_grid.addWidget(self.card_stt, 0, 3)
        n_layout.addLayout(n_grid)
        c_layout.addWidget(card_net)
        c_layout.addStretch(1)

        scroll.setWidget(container)
        layout.addWidget(scroll)

    def update_power_water(self, snap: PowerWaterSnapshot) -> None:
        """Update battery, voltage curve, and kettle metrics."""
        pct = snap.battery_percent
        self.lbl_bat_pct.setText(f"{pct:.0f}%")
        color = KioskTheme.COLOR_HEALTHY if pct >= 30 else (
            KioskTheme.COLOR_WARNING if pct >= 15 else KioskTheme.COLOR_CRITICAL
        )
        self.lbl_bat_pct.setStyleSheet(f"color: {color}; font-size: 32px; font-weight: 900;")
        self.pbar_bat.setValue(int(pct))
        self.pbar_bat.setStyleSheet(f"""
            QProgressBar {{ border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 4px; background: {KioskTheme.COLOR_SURFACE}; }}
            QProgressBar::chunk {{ background: {color}; border-radius: 3px; }}
        """)
        self.lbl_bat_sub.setText(f"Điện áp: {snap.voltage_smooth:.2f}V (Tức thời: {snap.voltage_raw:.2f}V) | Nguồn: {snap.voltage_source}")

        # Water
        water_pct = min(100.0, (snap.water_liter / 3.0) * 100.0) if snap.water_liter > 0 else 0.0
        self.lbl_water_val.setText(f"{snap.water_liter:.1f} L")
        kettle_ok = snap.kettle_present
        self.lbl_kettle_status.setText(f"Khay ấm: {'Đã đặt ấm trên đế [OK]' if kettle_ok else 'Chưa có ấm trên đế [CẢNH BÁO]'}")
        self.lbl_kettle_status.setStyleSheet(f"color: {'#22c55e' if kettle_ok else '#ef4444'}; font-size: 12px; font-weight: bold;")
        self.lbl_water_sub.setText(f"Mực nước đạt {water_pct:.0f}% dung tích bình chứa")

    def update_host_telemetry(self, snap: HostTelemetrySnapshot) -> None:
        """Update Jetson CPU, GPU, RAM, Disk, and Network telemetry."""
        # 1. CPU & GPU
        cpu_t = snap.cpu_temp
        cpu_st = HealthStatus.CRITICAL if cpu_t >= 85 else (HealthStatus.WARNING if cpu_t >= 75 else HealthStatus.HEALTHY)
        self.card_cpu.update_data(f"{snap.cpu_pct:.1f}%", f"Nhiệt độ: {cpu_t:.1f}°C", cpu_st)

        gpu_t = snap.gpu_temp
        gpu_st = HealthStatus.CRITICAL if gpu_t >= 85 else (HealthStatus.WARNING if gpu_t >= 75 else HealthStatus.HEALTHY)
        self.card_gpu.update_data(f"{snap.gpu_pct:.1f}%", f"Nhiệt độ: {gpu_t:.1f}°C", gpu_st)

        # Thermal notice
        if cpu_t >= 85.0:
            self.lbl_thermal_notice.setText(f"BÁO ĐỘNG: Nhiệt độ SoC quá cao ({cpu_t:.1f}°C)! Nguy cơ sập hệ thống.")
            self.lbl_thermal_notice.setStyleSheet(f"color: {KioskTheme.COLOR_CRITICAL}; font-size: 11px; font-weight: bold;")
        elif cpu_t >= 75.0:
            self.lbl_thermal_notice.setText(f"CẢNH BÁO: Nhiệt độ Jetson cao ({cpu_t:.1f}°C), hệ thống đang giảm xung nhịp.")
            self.lbl_thermal_notice.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_thermal_notice.setText(f"Nhiệt độ SoC an toàn ({cpu_t:.1f}°C < 75°C).")
            self.lbl_thermal_notice.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 11px; font-weight: bold;")

        # 2. RAM & Disk
        ram_st = HealthStatus.WARNING if snap.ram_pct >= 90 else HealthStatus.HEALTHY
        self.card_ram.update_data(f"{snap.ram_used_gb:.1f} / {snap.ram_total_gb:.1f} GB", f"{snap.ram_pct:.0f}% đã dùng", ram_st)

        disk_st = HealthStatus.WARNING if snap.disk_pct >= 90 else HealthStatus.HEALTHY
        self.card_disk.update_data(f"{snap.disk_free_gb:.1f} GB trống", f"{snap.disk_pct:.0f}% đã dùng", disk_st)

        # 3. Network & AI Services
        self.card_ip.update_data(snap.local_ip, "LAN / Wi-Fi", HealthStatus.HEALTHY)
        self.card_wifi.update_data(snap.wifi_ssid, f"Sóng: {snap.wifi_signal_pct}%", HealthStatus.HEALTHY)

        router_st = HealthStatus.HEALTHY if snap.router_ok else HealthStatus.CRITICAL
        router_val = "ONLINE" if snap.router_ok else "OFFLINE"
        self.card_router.update_data(router_val, f"Port {snap.router_port} · LLM & TTS", router_st)

        stt_st = HealthStatus.HEALTHY if snap.stt_ok else HealthStatus.CRITICAL
        stt_val = f"OK ({snap.stt_latency_ms}ms)" if snap.stt_ok else "LỖI API"
        self.card_stt.update_data(stt_val, "Groq Whisper", stt_st)
        self.card_cloud.update_data(stt_val, "Groq Whisper", stt_st)

        # 4. Process Monitor Table update
        if snap.top_processes:
            self._last_top_processes = snap.top_processes
        if hasattr(self, "table_proc") and not self.table_proc.isHidden() and self._last_top_processes:
            self._update_process_table(self._last_top_processes)

        # Update Header Health status
        if cpu_t >= 85.0 or snap.ram_pct >= 95.0:
            self.header.set_status("CRITICAL", HealthStatus.CRITICAL)
        elif cpu_t >= 75.0 or snap.ram_pct >= 85.0 or not snap.router_ok:
            self.header.set_status("CẢNH BÁO", HealthStatus.WARNING)
        elif cpu_t > 0:
            self.header.set_status("HEALTHY", HealthStatus.HEALTHY)
        else:
            self.header.set_status("ĐANG KHỞI TẠO", HealthStatus.UNKNOWN)

    def _toggle_proc_table(self) -> None:
        """Toggle Process Monitor visibility."""
        if not self.table_proc.isHidden():
            self.table_proc.hide()
            self.btn_toggle_proc.setText("Hiện Tiến Trình (Process Monitor)")
        else:
            self.table_proc.show()
            self.btn_toggle_proc.setText("Ẩn Tiến Trình (Process Monitor)")
            if self._last_top_processes:
                self._update_process_table(self._last_top_processes)

    def _update_process_table(self, top_processes: list[dict[str, Any]]) -> None:
        """Populate the process table with top CPU/RAM consuming tasks."""
        self.table_proc.setRowCount(len(top_processes))
        for row, p in enumerate(top_processes):
            pid_item = QTableWidgetItem(str(p.get("pid", 0)))
            pid_item.setTextAlignment(Qt.AlignCenter)
            self.table_proc.setItem(row, 0, pid_item)

            name_item = QTableWidgetItem(str(p.get("name", "unknown")))
            self.table_proc.setItem(row, 1, name_item)

            cpu_val = p.get("cpu", 0.0)
            cpu_item = QTableWidgetItem(f"{cpu_val:.1f}%")
            cpu_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            if cpu_val > 50.0:
                cpu_item.setForeground(QColor("#ef4444"))
            elif cpu_val > 20.0:
                cpu_item.setForeground(QColor("#f59e0b"))
            self.table_proc.setItem(row, 2, cpu_item)

            mem_val = float(p.get("mem_mb", p.get("ram_mb", 0.0)))
            mem_item = QTableWidgetItem(f"{mem_val:.1f} MB")
            mem_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table_proc.setItem(row, 3, mem_item)

            st_item = QTableWidgetItem(str(p.get("status", "running")).upper())
            st_item.setTextAlignment(Qt.AlignCenter)
            self.table_proc.setItem(row, 4, st_item)
