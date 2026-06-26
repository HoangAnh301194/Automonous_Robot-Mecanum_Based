#!/bin/bash
# Script cai dat thu vien can thiet cho YOLO ROS tren moi truong Native Jetson Orin Nano

set -e

WS_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "1. Cap nhat Repository va cai dat thu vien PyTorch toi uu cho JetPack..."
sudo apt-get update
sudo apt-get install -y python3-torch python3-torchvision python3-colcon-common-extensions

echo "2. Cai dat cac thu vien can thiet cua YOLO (Bo qua cac phu thuoc de bao ve PyTorch)..."
pip3 install ultralytics==8.4.6 numpy<2 opencv-python>=4.8.1.78 lap>=0.5.12 typing-extensions>=4.4.0 --no-deps

echo "3. Bien dich lai workspace ROS 2..."
cd "$WS_ROOT"
colcon build --packages-select yolo_ros yolo_msgs yolo_bringup

echo "--------------------------------------------------------"
echo "Cai dat thanh cong! Ban hay chay lenh sau de su dung YOLO chay bang he thong GPU:"
echo "source $WS_ROOT/install/setup.bash"
echo "ros2 launch yolo_bringup yolov8.launch.py"
echo "--------------------------------------------------------"
