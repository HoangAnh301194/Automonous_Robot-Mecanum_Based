#!/usr/bin/env bash
# ==============================================================================
# Script dọn dẹp và tắt sạch toàn bộ tiến trình ROS 2 trên Robot / Jetson Orin
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}>>> Đang kích hoạt dừng an toàn cho Robot...${NC}"

# 1. Phát lệnh phanh dừng bánh xe tức thì nếu ROS 2 đang chạy
if command -v ros2 &> /dev/null; then
    timeout 1 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true
fi

# 2. Tắt các tiến trình cụ thể bằng killall
echo -e "${BLUE}>>> Tắt UI, 9router, Nav2 và các tiến trình ROS 2...${NC}"
killall -9 ros2 _ros2_daemon rviz2 robot_ui_kiosk 9router 2>/dev/null || true

# 3. Tắt toàn diện bằng regex pkill
pkill -9 -f "(ros2|nav2|robot_|kiosk|9router|rviz2|obstacle_detector|astra_camera|sllidar|hoverboard|person_depth|gui_v3)" 2>/dev/null || true

# 4. Khởi động lại ROS 2 Daemon để tránh kẹt DDS endpoint
if command -v ros2 &> /dev/null; then
    ros2 daemon stop > /dev/null 2>&1 || true
    ros2 daemon start > /dev/null 2>&1 || true
fi

echo -e "${GREEN}======================================================${NC}"
echo -e "${GREEN}✅ ĐÃ DỌN DẸP VÀ TẮT SẠCH TOÀN BỘ TIẾN TRÌNH ROBOT!${NC}"
echo -e "${GREEN}======================================================${NC}"
