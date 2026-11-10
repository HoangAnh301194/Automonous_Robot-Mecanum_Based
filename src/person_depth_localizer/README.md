# Person Depth Localizer (uncalibrated preview)

**Test only: no Nav2 control, no automatic goals. RGB-Depth calibration is not performed.**

Inputs: RGB image dimensions, YOLO person /yolo/tracking, AI Depth image and CameraInfo, TF2.

Outputs:
- /mission/person_preview_base (geometry_msgs/PointStamped): planar x/y in base_link
- /mission/person_preview_map (geometry_msgs/PointStamped): planar x/y in map if TF exists
- /mission/person_distance_preview (std_msgs/Float32): planar distance
- /mission/person_marker (visualization_msgs/Marker): orange preview sphere
- /mission/person_marker_array (visualization_msgs/MarkerArray): sphere and label
- /mission/person_preview_status (std_msgs/String): JSON with calibrated=false and navigation_ready=false.

## Install

~~~bash
cd ~/ros2_ws
git switch simulator
git pull --ff-only origin simulator
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to person_depth_localizer
source install/setup.bash
~~~

Terminal 1 (stop existing camera launches first):

~~~bash
ros2 launch mo_hinh three_astra.launch.py
~~~

Terminal 2 (replace path with actual engine location):

~~~bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=/absolute/path/to/yolo11n.engine device:=cuda:0 \
  input_image_topic:=/ai_camera/color/image_raw \
  image_reliability:=2 imgsz_height:=384 imgsz_width:=640 \
  classes:=0 use_tracking:=True use_3d:=False
~~~

Before starting TF, check actual Depth header frame:

~~~bash
ros2 topic echo /ai_camera/depth/image_raw --field header --once
~~~

Terminal 3, ONLY if the Depth frame is named ai_camera_depth_optical_frame and its transform does not already exist:

~~~bash
ros2 launch person_depth_localizer rough_person_preview.launch.py \
  publish_test_tf:=true \
  test_parent_frame:=base_footprint \
  test_x:=0 test_y:=0 test_z:=1.5 \
  depth_frame:=ai_camera_depth_optical_frame
~~~

The mock TF above assumes camera optical centre is directly above the base_footprint at 1.5 m and points horizontally forward. It must NOT be used for navigation. If a proper sensor TF already exists, launch with publish_test_tf:=false. If the actual Depth frame differs, configure depth_frame to the real header.frame_id.

Verify:

~~~bash
ros2 topic echo /mission/person_preview_status
ros2 topic echo /mission/person_preview_base --once
ros2 topic echo /mission/person_distance_preview --once
ros2 topic echo /mission/person_preview_map --once
ros2 run tf2_ros tf2_echo base_link ai_camera_depth_optical_frame
~~~

RViz2: Fixed Frame map (if AMCL/localization running) or base_link; Add MarkerArray, topic /mission/person_marker_array.

## Limitations and next calibration stages

1. RGB bbox pixels are scaled to Depth dimensions using a normalized image ratio, NOT valid camera registration. Camera parallax/FOV differences can select background depths. Bench test with just one person in the centre of the view.
2. If Depth CameraInfo.K is zero, a rough assumed field of view is used. Every output remains uncalibrated.
3. Depth and tracking must be within 0.25 seconds by header timestamp. Failed pairing or TF produces status but no person position. Marker lifetime is one second.
4. RGB camera calibration: use an 8x6 inner-corner checkerboard at known real square size, collect multiple poses, save the camera_info YAML, and configure usb_cam to load it.
5. Depth: verify K and distortion at 160x120. RGB/Depth extrinsics require a common visible target or validated Orbbec registration, not a simple x4 pixel mapping.
6. Measure camera optical pose x/y/z and roll/pitch/yaw relative to the robot. Replace mock transform with calibrated static TF/URDF, validate on measured targets, then implement robust per-track association and safe target stopping distances.
7. Never feed this preview position to Nav2 or the Behavior Tree as a navigation goal.
