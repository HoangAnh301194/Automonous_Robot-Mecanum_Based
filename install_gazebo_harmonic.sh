#!/bin/bash

echo "Bắt đầu cài đặt Gazebo Harmonic (Sim 8) cho Ubuntu 22.04..."

# 1. Cập nhật và cài đặt các công cụ cần thiết
echo "Cài đặt curl, lsb-release và gnupg..."
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg

# 2. Thêm khóa GPG và kho lưu trữ của OSRF
echo "Thêm kho lưu trữ của Gazebo..."
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Cập nhật lại kho lưu trữ và cài đặt Gazebo Harmonic cùng gói cầu nối ROS 2 Humble
echo "Cài đặt gz-harmonic và ros-humble-ros-gzharmonic..."
sudo apt-get update
sudo apt-get install -y gz-harmonic ros-humble-ros-gzharmonic

echo "========================================================"
echo "Cài đặt hoàn tất! Bạn có thể kiểm tra bằng lệnh:"
echo "gz sim"
echo "Lưu ý: Để dùng với ROS 2, hãy chạy lệnh sau trước khi chạy node:"
echo "export GZ_VERSION=harmonic"
echo "========================================================"
