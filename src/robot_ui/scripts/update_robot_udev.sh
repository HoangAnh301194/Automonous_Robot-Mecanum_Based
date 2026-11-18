#!/usr/bin/env bash
# ==============================================================================
# Script tự động cập nhật udev rules cố định cổng RPLiDAR và Hoverboard/ESP32
# Sử dụng thuộc tính đường dẫn bus vật lý ID_PATH để chống xung đột chip CP2102.
# ==============================================================================
set -euo pipefail

LIDAR_DEV="${1:-}"
HOVER_DEV="${2:-}"

if [ -z "$LIDAR_DEV" ] || [ -z "$HOVER_DEV" ]; then
    echo "Usage: $0 <lidar_device> <hoverboard_device>"
    echo "Example: $0 /dev/ttyUSB1 /dev/ttyUSB0"
    exit 1
fi

# Chuyển đổi symlink về canonical device (ví dụ /dev/ttyUSB1)
LIDAR_CANON=$(readlink -f "$LIDAR_DEV" || echo "$LIDAR_DEV")
HOVER_CANON=$(readlink -f "$HOVER_DEV" || echo "$HOVER_DEV")

if [ ! -e "$LIDAR_CANON" ]; then
    echo "Error: Device LiDAR không tồn tại: $LIDAR_CANON" >&2
    exit 2
fi

if [ ! -e "$HOVER_CANON" ]; then
    echo "Error: Device Hoverboard không tồn tại: $HOVER_CANON" >&2
    exit 2
fi

# Lấy ID_PATH từ udevadm
LIDAR_PATH=$(udevadm info -q property -n "$LIDAR_CANON" | grep '^ID_PATH=' | cut -d'=' -f2 || true)
HOVER_PATH=$(udevadm info -q property -n "$HOVER_CANON" | grep '^ID_PATH=' | cut -d'=' -f2 || true)

if [ -z "$LIDAR_PATH" ] || [ -z "$HOVER_PATH" ]; then
    echo "Error: Không tìm thấy ID_PATH (LiDAR: '$LIDAR_PATH', Hoverboard: '$HOVER_PATH')" >&2
    exit 3
fi

if [ "$LIDAR_PATH" = "$HOVER_PATH" ]; then
    echo "Error: Hai thiết bị đang trỏ vào cùng 1 cổng vật lý ($LIDAR_PATH)!" >&2
    exit 4
fi

RULES_FILE="/etc/udev/rules.d/99-robot-serial.rules"
TMP_FILE=$(mktemp /tmp/99-robot-serial.rules.XXXXXX)

cat << EOF > "$TMP_FILE"
# ==============================================================================
# CỐ ĐỊNH CỔNG SERIAL CHO ROBOT (TỰ ĐỘNG TẠO BỞI ROBOT KIOSK UI)
# Thời gian cập nhật: $(date)
# ==============================================================================

# 1. RPLiDAR ($LIDAR_CANON tại $LIDAR_PATH)
SUBSYSTEM=="tty", ENV{ID_PATH}=="$LIDAR_PATH", MODE:="0777", SYMLINK+="rplidar"

# 2. Bo Hoverboard / ESP32 ($HOVER_CANON tại $HOVER_PATH)
SUBSYSTEM=="tty", ENV{ID_PATH}=="$HOVER_PATH", MODE:="0777", SYMLINK+="hoverboard"
EOF

# Ghi đè vào /etc/udev/rules.d/ (nếu cần sudo)
if [ -w "/etc/udev/rules.d" ]; then
    mv "$TMP_FILE" "$RULES_FILE"
    chmod 644 "$RULES_FILE"
else
    sudo mv "$TMP_FILE" "$RULES_FILE"
    sudo chmod 644 "$RULES_FILE"
fi

# Nạp lại udev rules và trigger ngay lập tức
if command -v udevadm >/dev/null 2>&1; then
    if [ "$(id -u)" -eq 0 ]; then
        udevadm control --reload-rules
        udevadm trigger -c add -s tty
    else
        sudo udevadm control --reload-rules
        sudo udevadm trigger -c add -s tty
    fi
fi

echo "SUCCESS: Đã cố định udev thành công:"
echo "  LiDAR      -> /dev/rplidar    (Gốc: $LIDAR_CANON, ID_PATH: $LIDAR_PATH)"
echo "  Hoverboard -> /dev/hoverboard (Gốc: $HOVER_CANON, ID_PATH: $HOVER_PATH)"
