#!/usr/bin/env python3
"""Static regression checks for ROS 2 AI camera-topic routing.

Run from anywhere: python3 tools/test_ai_topic_contract.py
Does not require ROS, USB cameras, or Python AI dependencies.
"""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def source(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class AITopicContractTests(unittest.TestCase):
    def test_three_camera_launch_names_remain_stable(self):
        text = source("src/mo_hinh/launch/three_astra.launch.py")
        self.assertIn("namespace='ai_camera/color'", text)
        self.assertIn("depth_camera('front_camera'", text)
        self.assertIn("depth_camera('rear_camera'", text)
        self.assertIn("'enable_color': False", text)
        self.assertIn("'enable_depth': True", text)

    def test_yolo_all_nodes_remap_to_ai_rgb(self):
        text = source("src/yolo_ros/yolo_bringup/launch/yolo.launch.py")
        self.assertIn('default_value="/ai_camera/color/image_raw"', text)
        self.assertIn('"image_reliability",\n            default_value="2"', text)
        self.assertEqual(text.count('remappings=[("image_raw", input_image_topic)]'), 2)
        self.assertIn('("image_raw", input_image_topic)', text)
        self.assertIn('default_value=""', text)
        self.assertNotIn('default_value="/camera/color/image_raw"', text)

    def test_rtmpose_defaults_are_ai_rgb_cpu_and_tracking(self):
        launch = source("src/HandWaveDetection_Pose/launch/hand_wave_detection.launch.py")
        node = source("src/HandWaveDetection_Pose/hand_wave_detection/ros_node.py")
        self.assertIn("default_value='/ai_camera/color/image_raw'", launch)
        self.assertIn("default_value='cpu'", launch)
        self.assertIn("default_value='/yolo/tracking'", launch)
        self.assertIn("'image_topic', '/ai_camera/color/image_raw'", node)
        self.assertIn("'tracking_topic', '/yolo/tracking'", node)

    def test_person_follower_does_not_assume_registered_depth(self):
        text = source("src/yolo_ros/yolo_bringup/launch/person_follower.launch.py")
        self.assertIn("default_value='/ai_camera/color/image_raw'", text)
        self.assertIn("'use_3d', default_value='False'", text)
        self.assertIn("condition=IfCondition(LaunchConfiguration('use_3d'))", text)
        self.assertNotIn("'use_3d': 'True'", text)

    def test_one_command_pipeline_consistent_and_safe(self):
        text = source("start_person_following.sh")
        for expected in (
            "three_astra.launch.py",
            'AI_IMAGE_TOPIC="/ai_camera/color/image_raw"',
            'TRACKING_TOPIC="/yolo/tracking"',
            'input_image_topic:="$AI_IMAGE_TOPIC"',
            'image_topic:="$AI_IMAGE_TOPIC"',
            'tracking_topic:="$TRACKING_TOPIC"',
            "use_tracking:=True",
            "use_3d:=False",
            'HAND_WAVE_DEVICE="${HAND_WAVE_DEVICE:-cpu}"',
        ):
            self.assertIn(expected, text)
        self.assertNotIn("pkill", text)
        self.assertNotIn("rm -rf /dev/shm", text)
        self.assertNotIn("astra_pro.launch.xml", text)

    def test_ui_monitors_correct_three_camera_info_topics(self):
        files = (
            "src/robot_ui/config/robot_ui.yaml",
            "src/robot_ui/robot_ui/config.py",
            "src/robot_ui/frontend/src/pages/VisionPage.tsx",
        )
        for path in files:
            text = source(path)
            with self.subTest(path=path):
                self.assertIn("/ai_camera/color/camera_info", text)
                self.assertIn("/front_camera/depth/camera_info", text)
                self.assertIn("/rear_camera/depth/camera_info", text)
                self.assertNotIn("/camera/color/camera_info", text)
        bridge = source("src/robot_ui/robot_ui/ros_bridge.py")
        self.assertIn('self._topics["camera_rear_depth_info"]', bridge)
        self.assertIn("self._on_rear_depth_camera_info", bridge)


if __name__ == "__main__":
    unittest.main()
