#!/bin/bash

# Function to clean up background processes when Ctrl+C is pressed
cleanup() {
    echo ""
    echo "Caught Ctrl+C! Terminating all background processes..."
    kill $(jobs -p) 2>/dev/null
    # Also attempt to run the original cleanup commands just in case
    pkill -9 -f ros2 2>/dev/null
    pkill -9 -f astra_camera 2>/dev/null
    pkill -9 -f yolo_node 2>/dev/null
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM


# 1. Clean up old processes
echo "Cleaning up old processes and shared memory..."
pkill -9 -f ros2
pkill -9 -f astra_camera
pkill -9 -f realsense2_camera
pkill -9 -f yolo_node
pkill -9 -f follower_node
pkill -9 -f face_recognition_node
pkill -9 -f pose_ros_node

rm -rf /dev/shm/fastrtps* 2>/dev/null
sleep 2

# 2. Reset ROS Environment
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset COLCON_PREFIX_PATH

# 3. Source Workspace
source /opt/ros/humble/setup.bash
cd /home/orin/ros2_ws
source install/setup.bash

# 4. Launch Astra Pro Camera in background
# Topics: /camera/color/image_raw, /camera/depth/image_raw, /camera/depth/camera_info
echo "Launching Astra Pro Camera..."
ros2 launch astra_camera astra_pro.launch.xml \
    camera_name:=camera \
    enable_color:=true \
    enable_depth:=true \
    color_width:=640 color_height:=480 color_fps:=30 \
    depth_width:=640 depth_height:=480 depth_fps:=30 \
    > /tmp/camera.log 2>&1 &
sleep 5

# 5. Launch Person Follower (YOLOv8 TensorRT)
echo "Launching Person Follower (YOLO26 pose TensorRT)..."
ros2 launch yolo_bringup person_follower.launch.py \
    model:=/home/orin/ros2_ws/src/Pose_detection/yolo26n-pose.engine \
    device:=cuda:0 \
    input_image_topic:=/camera/color/image_raw \
    input_depth_topic:=/camera/depth/image_raw \
    input_depth_info_topic:=/camera/depth/camera_info \
    > /tmp/yolo.log 2>&1 &
sleep 15

# 6. Launch Pose Detection (Vẫy tay)
echo "Launching Pose Detection (Wave Gesture)..."
ros2 run pose_detection pose_ros_node &
sleep 2

echo "----------------------------------------------------"
echo "He thong da san sang! (Face Recognition da tat)"
echo "Camera: Orbbec Astra Pro"
echo "  - Color: /camera/color/image_raw"
echo "  - Depth: /camera/depth/image_raw"
echo "- Xem anh Debug YOLO: ros2 run rqt_image_view rqt_image_view /yolo/dbg_image"
echo "- Trang thai vay tay: ros2 topic echo /pose/wave_status"
echo "- Log camera:  tail -f /tmp/camera.log"
echo "- Log YOLO:    tail -f /tmp/yolo.log"
echo "----------------------------------------------------"

# Stay alive to keep processes running
wait
