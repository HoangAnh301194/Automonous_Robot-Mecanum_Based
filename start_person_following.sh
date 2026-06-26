#!/bin/bash

# 1. Clean up old processes
echo "Cleaning up old processes and shared memory..."
pkill -9 -f ros2
pkill -9 -f realsense2_camera
pkill -9 -f yolo_node
pkill -9 -f follower_node
pkill -9 -f face_recognition_node
pkill -9 -f pose_ros_node

rm -rf /dev/shm/fastrtps* 2>/dev/null

# 2. Reset ROS Environment
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset COLCON_PREFIX_PATH

# 3. Source Workspace MỚI
source /opt/ros/humble/setup.bash
cd /home/orin/ros2_ws
source install/setup.bash

# 4. Launch the Camera in background
echo "Launching Realsense Camera..."
# Redirect camera to log file instead of /dev/null
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true > /tmp/camera.log 2>&1 &
sleep 5

# 5. Launch the Person Follower (YOLOv8) với TensorRT
echo "Launching Person Follower (YOLOv8 TensorRT)..."
# GIỮ LOG RA TERMINAL ĐỂ KIỂM TRA LỖI
ros2 launch yolo_bringup person_follower.launch.py model:=/home/orin/ros2_ws/src/Pose_detection/yolov8n-pose.engine &
sleep 15

# 6. Launch Pose Detection (Vẫy tay)
echo "Launching Pose Detection (Wave Gesture)..."
ros2 run pose_detection pose_ros_node &
sleep 2

# 7. Launch Face Recognition (Optimized)
echo "Launching Face Recognition (Optimized)..."
ros2 run face_recognition_yolo face_recognition_node \
    --ros-args -p process_every_n_frames:=5 \
    -p spam_cooldown:=3.0 > /tmp/face.log 2>&1 &
sleep 5

echo "----------------------------------------------------"
echo "Hệ thống đã sẵn sàng!"
echo "Current Prefix: $(ros2 pkg prefix yolo_bringup)"
echo "- Xem ảnh Debug YOLO: ros2 run rqt_image_view rqt_image_view /yolo/dbg_image"
echo "- Trạng thái vẫy tay: ros2 topic echo /pose/wave_status"
echo "----------------------------------------------------"

# Stay alive to keep processes running
wait
