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

# 2. Gửi tín hiệu ngắt êm (SIGINT -2) để driver Camera Astra / USB đóng stream và tắt đèn phần cứng
echo -e "${BLUE}>>> Gửi tín hiệu đóng stream Camera và dừng các dịch vụ...${NC}"
pkill -2 -f "(astra|usb_cam|camera|robot_|kiosk|gui_v3)" 2>/dev/null || true
sleep 0.6

# 3. Tắt dứt điểm toàn bộ các binary cụ thể
echo -e "${BLUE}>>> Buộc dừng (SIGKILL -9) toàn bộ tiến trình ROS 2, Camera, AI và Nav2...${NC}"
killall -9 \
    ros2 \
    _ros2_daemon \
    rviz2 \
    robot_ui_kiosk \
    9router \
    astra_camera_node \
    usb_cam_node_exe \
    static_transform_publisher \
    yolo_node \
    tracking_node \
    rough_localizer \
    hand_wave_detector \
    obstacle_detector_node \
    depthimage_to_laserscan_node \
    rf2o_laser_odometry_node \
    sllidar_node \
    lifecycle_manager \
    bt_navigator \
    controller_server \
    planner_server \
    behavior_server \
    smoother_server \
    waypoint_follower \
    amcl \
    slam_toolbox 2>/dev/null || true

# 4. Quét toàn diện các tiến trình liên quan bằng pkill regex
pkill -9 -f "(ros2|nav2|robot_|kiosk|9router|rviz2|obstacle_detector|astra|usb_cam|sllidar|hoverboard|person_depth|gui_v3|yolo|transform_publisher|depthimage|laser_odometry|tracking_node|hand_wave|rf2o)" 2>/dev/null || true

# 5. Khởi động lại ROS 2 Daemon để giải phóng hoàn toàn DDS endpoints
if command -v ros2 &> /dev/null; then
    ros2 daemon stop > /dev/null 2>&1 || true
    ros2 daemon start > /dev/null 2>&1 || true
fi

echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}✅ ĐÃ DỌN DẸP VÀ TẮT SẠCH TOÀN BỘ TIẾN TRÌNH ROBOT & CAMERA!${NC}"
echo -e "${GREEN}================================================================${NC}"
