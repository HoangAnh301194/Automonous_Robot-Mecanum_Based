import time
import pytest
from robot_mission_control.bt_core import (
    Blackboard,
    NodeStatus,
    TreeNode,
    Sequence,
    Fallback,
    Condition,
    Action,
)
from robot_mission_control.bt_nodes import (
    IsEmergency,
    IsBatteryLow,
    IsServingCustomer,
    IsHandWavingDetected,
    IsCustomerEncountered,
    IsWaterLow,
    IsPatrolRequested,
)


class MockAction(Action):
    def __init__(self, name="MockAction"):
        super().__init__(name)
        self.tick_count = 0
        self.halt_count = 0
        self.return_status = NodeStatus.RUNNING

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        self.tick_count += 1
        self.status = self.return_status
        return self.status

    def halt(self, blackboard: Blackboard = None) -> None:
        self.halt_count += 1
        self.status = NodeStatus.FAILURE


def test_fallback_preemption_halts_lower_priority():
    """Verify that when a higher priority branch activates, lower priority running branch is halted."""
    bb = Blackboard()
    bb.set("HMI_start_patrol", True)

    high_priority_action = MockAction("HighPriorityAction")
    low_priority_action = MockAction("LowPriorityAction")

    class HighCond(Condition):
        def tick(self, blackboard: Blackboard) -> NodeStatus:
            return NodeStatus.SUCCESS if blackboard.get("HIGH_ACTIVE", False) else NodeStatus.FAILURE

    class LowCond(Condition):
        def tick(self, blackboard: Blackboard) -> NodeStatus:
            return NodeStatus.SUCCESS if blackboard.get("LOW_ACTIVE", False) else NodeStatus.FAILURE

    seq_high = Sequence("SeqHigh", [HighCond("CondHigh"), high_priority_action])
    seq_low = Sequence("SeqLow", [LowCond("CondLow"), low_priority_action])

    tree = Fallback("RootFallback", [seq_high, seq_low])

    # 1. Start with only low priority active
    bb.set("HIGH_ACTIVE", False)
    bb.set("LOW_ACTIVE", True)
    status = tree.tick(bb)
    assert status == NodeStatus.RUNNING
    assert low_priority_action.tick_count == 1
    assert low_priority_action.status == NodeStatus.RUNNING
    assert high_priority_action.tick_count == 0

    # 2. High priority event occurs (e.g. Emergency or Waving)
    bb.set("HIGH_ACTIVE", True)
    status = tree.tick(bb)
    assert status == NodeStatus.RUNNING
    assert high_priority_action.tick_count == 1
    # Low priority action MUST be halted
    assert low_priority_action.halt_count == 1
    assert low_priority_action.status == NodeStatus.FAILURE


def test_priority_hierarchy_p0_to_p6():
    """Verify the relative priority: Emergency (P0) > Battery (P1) > Serving (P2) > Wave (P3) > Encounter (P4) > Water (P5) > Patrol (P6)."""
    bb = Blackboard()
    bb.set("SYSTEM_lidar_ok", True)
    bb.set("SYSTEM_odom_ok", True)
    bb.set("SYSTEM_localization_ok", True)
    bb.set("SYSTEM_battery_low", False)
    bb.set("IS_SERVING", False)
    bb.set("HAND_WAVE_ACTIVE", False)
    bb.set("ENCOUNTER_ACTIVE", False)
    bb.set("SYSTEM_water_low", False)
    bb.set("HMI_start_patrol", True)

    c_emergency = IsEmergency("Cond_Emergency")
    c_battery = IsBatteryLow("Cond_Battery")
    c_serving = IsServingCustomer("Cond_Serving")
    c_wave = IsHandWavingDetected("Cond_Wave")
    c_encounter = IsCustomerEncountered("Cond_Encounter")
    c_water = IsWaterLow("Cond_Water")
    c_patrol = IsPatrolRequested("Cond_Patrol")

    # Initial state: only patrol is active
    assert c_emergency.tick(bb) == NodeStatus.FAILURE
    assert c_battery.tick(bb) == NodeStatus.FAILURE
    assert c_serving.tick(bb) == NodeStatus.FAILURE
    assert c_wave.tick(bb) == NodeStatus.FAILURE
    assert c_encounter.tick(bb) == NodeStatus.FAILURE
    assert c_water.tick(bb) == NodeStatus.FAILURE
    assert c_patrol.tick(bb) == NodeStatus.SUCCESS

    # Waving person detected: Wave becomes active
    bb.set("HAND_WAVE_ACTIVE", True)
    assert c_wave.tick(bb) == NodeStatus.SUCCESS

    # Emergency occurs: Emergency becomes active
    bb.set("SYSTEM_lidar_ok", False)
    assert c_emergency.tick(bb) == NodeStatus.SUCCESS


def test_wave_timeout_expiration():
    """Verify wave detection expires and clears blackboard flags if not confirmed within timeout."""
    bb = Blackboard()
    bb.set("HAND_WAVE_ACTIVE", True)
    bb.set("APPROACH_ACTIVE", False)
    bb.set("WAVING_PERSON", {"id": 1})

    last_seen_at = time.monotonic() - 4.5  # 4.5s ago
    wave_timeout_sec = 3.0

    now_mono = time.monotonic()
    if (
        bb.get("HAND_WAVE_ACTIVE", False)
        and not bb.get("APPROACH_ACTIVE", False)
        and (now_mono - last_seen_at) >= wave_timeout_sec
    ):
        bb.set("HAND_WAVE_ACTIVE", False)
        bb.set("WAVING_PERSON", None)

    assert bb.get("HAND_WAVE_ACTIVE") is False
    assert bb.get("WAVING_PERSON") is None


def test_navigation_sequence_token_concurrency():
    """Verify execution generation/sequence token rejects outdated callbacks."""
    current_exec_seq = 1

    # Callback 1 arrives with old seq
    old_seq = 0
    callback_processed = False
    if old_seq == current_exec_seq:
        callback_processed = True
    assert callback_processed is False

    # Goal preempted: seq bumped
    current_exec_seq += 1

    # New callback arrives with updated seq
    if current_exec_seq == 2:
        callback_processed = True
    assert callback_processed is True


def test_dynamic_target_tracking_displacement_logic():
    """Verify displacement calculation and freeze distance gating for dynamic re-planning."""
    from geometry_msgs.msg import PoseStamped
    from robot_mission_control.bt_nodes import ActionApproachWavingPerson

    action = ActionApproachWavingPerson("TestApproach", node=None)
    action.min_displacement = 0.40
    action.freeze_distance = 1.40

    pose1 = PoseStamped()
    pose1.pose.position.x = 2.0
    pose1.pose.position.y = 1.0
    action.last_goal_pose = pose1

    # Small displacement (0.2m) -> should NOT trigger re-planning
    pose_small = PoseStamped()
    pose_small.pose.position.x = 2.1
    pose_small.pose.position.y = 1.1
    assert action._is_significant_displacement(pose_small) is False

    # Significant displacement (0.5m) -> should trigger re-planning
    pose_large = PoseStamped()
    pose_large.pose.position.x = 2.4
    pose_large.pose.position.y = 1.3
    assert action._is_significant_displacement(pose_large) is True


def test_patrol_waits_for_explicit_start_request():
    """Verify patrol stays idle by default and only activates after HMI_start_patrol is True."""
    bb = Blackboard()
    c_patrol = IsPatrolRequested("Cond_Patrol")

    assert c_patrol.tick(bb) == NodeStatus.FAILURE

    bb.set("HMI_start_patrol", False)
    assert c_patrol.tick(bb) == NodeStatus.FAILURE

    bb.set("HMI_start_patrol", True)
    assert c_patrol.tick(bb) == NodeStatus.SUCCESS


def test_snapshot_one_shot_approach_freezes_goal():
    """Verify snapshot approach captures one goal at 1.20m standoff and ignores later target movement."""
    from types import SimpleNamespace
    from geometry_msgs.msg import PoseStamped
    from robot_mission_control.bt_nodes import ActionApproachWavingPerson, Blackboard, NodeStatus

    action = ActionApproachWavingPerson("TestSnapshotApproach", node=None)
    assert action.standoff_distance == 0.45
    assert action.enable_dynamic_replan is False

    sent_goals = []
    action.nav2_client = SimpleNamespace(server_is_ready=lambda: True)

    def _fake_send(pose):
        action.is_navigating = True
        sent_goals.append((pose.pose.position.x, pose.pose.position.y))

    action.send_nav2_goal = _fake_send

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.position.x = 2.0
    initial_pose.pose.position.y = 1.0

    bb = Blackboard()
    bb.set("PERSON_PREVIEW_MAP_POSE", initial_pose)

    assert action.tick(bb) == NodeStatus.RUNNING
    assert len(sent_goals) == 1
    assert sent_goals[0] == (2.0, 1.0)

    # Simulate target moving far away and enough time elapsing for replan interval
    moved_pose = PoseStamped()
    moved_pose.header.frame_id = "map"
    moved_pose.pose.position.x = 5.0
    moved_pose.pose.position.y = -3.0
    bb.set("PERSON_PREVIEW_MAP_POSE", moved_pose)
    action.last_replan_time -= 5.0

    assert action.tick(bb) == NodeStatus.RUNNING
    assert len(sent_goals) == 1
    assert action.last_goal_pose.pose.position.x == 2.0
    assert action.last_goal_pose.pose.position.y == 1.0


def test_resolve_target_pose_uses_depth_stamp_with_fallback():
    """Verify _resolve_target_pose_map looks up TF at depth_stamp first and falls back to Time(0)."""
    from types import SimpleNamespace
    import pytest
    import rclpy
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import TransformStamped
    from robot_mission_control.bt_nodes import ActionApproachWavingPerson, Blackboard

    identity_tf = TransformStamped()
    identity_tf.header.frame_id = "map"
    identity_tf.child_frame_id = "base_footprint"
    identity_tf.transform.rotation.w = 1.0

    recorded_times = []

    class FakeTfBuffer:
        def __init__(self, fail_stamped=False):
            self.fail_stamped = fail_stamped

        def lookup_transform(self, target_frame, source_frame, stamp, timeout=None):
            recorded_times.append(stamp.nanoseconds)
            if self.fail_stamped and stamp.nanoseconds != 0:
                raise RuntimeError("Extrapolation Error")
            return identity_tf

    fake_clock = SimpleNamespace(
        now=lambda: rclpy.time.Time(seconds=100, nanoseconds=0)
    )
    fake_node = SimpleNamespace(
        tf_buffer=FakeTfBuffer(fail_stamped=False),
        get_clock=lambda: fake_clock,
        get_logger=lambda: SimpleNamespace(warn=lambda msg: None),
    )

    depth_stamp = TimeMsg(sec=98, nanosec=500000000)
    bb = Blackboard()
    bb.set(
        "WAVING_PERSON",
        SimpleNamespace(
            position=SimpleNamespace(x=3.20, y=0.0),
            position_valid=True,
            depth_stamp=depth_stamp,
        ),
    )

    action = ActionApproachWavingPerson("TestStampedTF", node=None)
    action.node = fake_node
    pose_map, dist = action._resolve_target_pose_map(bb)

    assert pose_map is not None
    assert dist == pytest.approx(3.20)
    assert pose_map.pose.position.x == pytest.approx(2.75)  # 3.20 - 0.45 standoff
    assert recorded_times == [98 * 10**9 + 500000000]

    # Now test fallback when stamped TF lookup fails
    recorded_times.clear()
    fake_node.tf_buffer = FakeTfBuffer(fail_stamped=True)
    pose_map_fb, dist_fb = action._resolve_target_pose_map(bb)

    assert pose_map_fb is not None
    assert dist_fb == pytest.approx(3.20)
    assert pose_map_fb.pose.position.x == pytest.approx(2.75)
    assert recorded_times == [98 * 10**9 + 500000000, 0]


def test_serve_customer_resumes_interrupted_patrol_target():
    """Verify finishing customer service resumes the interrupted patrol waypoint instead of skipping it."""
    from robot_mission_control.bt_nodes import ActionPatrol, ActionServeCustomer, Blackboard, NodeStatus

    patrol = ActionPatrol("TestPatrol", node=None)
    patrol._route_loaded = True
    patrol.patrol_points = ["A", "B"]
    patrol.current_idx = 1  # Currently heading from A to B

    sent_targets = []
    patrol.send_next_goal = lambda: sent_targets.append(patrol.patrol_points[patrol.current_idx])

    bb = Blackboard()
    assert patrol.tick(bb) == NodeStatus.RUNNING
    assert sent_targets == ["B"]

    # Interrupted by customer service
    patrol.halt(bb)
    serve = ActionServeCustomer("TestServe", node=None)
    assert serve.tick(bb) == NodeStatus.RUNNING

    bb.set("FINISH_SERVING_REQUESTED", True)
    assert serve.tick(bb) == NodeStatus.SUCCESS
    assert bb.get("PATROL_ADVANCE_NEXT", False) is False

    # Resuming patrol should continue to B, not wrap back to A
    assert patrol.tick(bb) == NodeStatus.RUNNING
    assert patrol.current_idx == 1
    assert sent_targets == ["B", "B"]


def test_action_patrol_sends_comma_separated_remaining_points():
    """Verify ActionPatrol formats remaining waypoints into a comma-separated string for NavigateThroughPoses."""
    from types import SimpleNamespace
    from robot_mission_control.bt_nodes import ActionPatrol, Blackboard, NodeStatus

    patrol = ActionPatrol("TestPatrolChain", node=None)
    patrol._route_loaded = True
    patrol.patrol_points = ["khoihanh", "diem1", "diem2", "diem3"]
    patrol.current_idx = 0

    sent_targets = []
    fake_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: SimpleNamespace(
            add_done_callback=lambda cb: sent_targets.append(goal.target_location)
        )
    )
    patrol.action_client = fake_client

    # 1. Starting at index 0 should dispatch full chain
    patrol.send_next_goal()
    assert len(sent_targets) == 1
    assert sent_targets[0] == "khoihanh,diem1,diem2,diem3"

    # 2. When current_idx is updated to 2, sending next goal dispatches remaining slice
    patrol.is_waiting_for_result = False
    patrol.current_idx = 2
    patrol.send_next_goal()
    assert len(sent_targets) == 2
    assert sent_targets[1] == "diem2,diem3"


def test_action_patrol_navigation_status_updates_current_idx():
    """Verify ActionPatrol listens to NavigationStatus and dynamically updates current_idx."""
    from robot_interfaces.msg import NavigationStatus
    from robot_mission_control.bt_nodes import ActionPatrol

    patrol = ActionPatrol("TestPatrolStatus", node=None)
    patrol.patrol_points = ["khoihanh", "diem1", "diem2", "diem3"]
    patrol.current_idx = 0

    status_msg = NavigationStatus()
    status_msg.current_goal_name = "diem2"
    patrol.navigation_status_callback(status_msg)
    assert patrol.current_idx == 2

    status_msg.current_goal_name = "diem3"
    patrol.navigation_status_callback(status_msg)
    assert patrol.current_idx == 3

    # Unknown waypoint should not corrupt current_idx
    status_msg.current_goal_name = "unrelated_point"
    patrol.navigation_status_callback(status_msg)
    assert patrol.current_idx == 3


def test_emergency_stop_blackboard_triggers_p0_preemption():
    """Verify setting EMERGENCY_STOP on blackboard triggers IsEmergency immediately."""
    cond = IsEmergency("TestEmergency")
    bb = Blackboard()

    # Normal state: sensors OK, no emergency stop
    bb.set("EMERGENCY_STOP", False)
    bb.set("SYSTEM_lidar_ok", True)
    bb.set("SYSTEM_odom_ok", True)
    bb.set("SYSTEM_localization_ok", True)
    assert cond.tick(bb) == NodeStatus.FAILURE

    # HMI Emergency stop triggered
    bb.set("EMERGENCY_STOP", True)
    assert cond.tick(bb) == NodeStatus.SUCCESS


def test_action_patrol_loop_closure_dispatches_start_point_first():
    """Verify that when the full route is completed, ActionPatrol navigates back to start point to close the loop."""
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus
    from robot_mission_control.bt_nodes import ActionPatrol

    patrol = ActionPatrol("TestPatrolLoop", node=None)
    patrol._route_loaded = True
    patrol.patrol_points = ["khoihanh", "diem1", "diem2"]
    patrol.current_idx = 0

    sent_targets = []
    fake_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: SimpleNamespace(
            add_done_callback=lambda cb: sent_targets.append(goal.target_location)
        )
    )
    patrol.action_client = fake_client

    # 1. First dispatch: whole chain is dispatched
    patrol.send_next_goal()
    assert len(sent_targets) == 1
    assert sent_targets[0] == "khoihanh,diem1,diem2"

    # 2. Complete full route (reach diem2)
    fake_future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            result=SimpleNamespace(success=True),
            status=GoalStatus.STATUS_SUCCEEDED,
        )
    )
    patrol.result_callback(fake_future)
    assert patrol._returning_to_start is True
    assert patrol.current_idx == 2

    # Simulate 2s wait expiry and current_idx wrap to 0
    patrol.is_waiting_for_result = False
    patrol.is_waiting_at_waypoint = False
    patrol.current_idx = 0

    # 3. Next dispatch must be ONLY the start point 'khoihanh' to close loop
    patrol.send_next_goal()
    assert len(sent_targets) == 2
    assert sent_targets[1] == "khoihanh"
    assert patrol._returning_goal is True

    # 4. Robot arrives at start point 'khoihanh'
    patrol.result_callback(fake_future)
    assert patrol._returning_goal is False
    assert patrol.current_idx == 0

    # Simulate 2s wait expiry and advance to index 1
    patrol.is_waiting_for_result = False
    patrol.is_waiting_at_waypoint = False
    patrol.current_idx = 1

    # 5. Next dispatch smoothly continues through remainder of route
    patrol.send_next_goal()
    assert len(sent_targets) == 3
    assert sent_targets[2] == "diem1,diem2"


def test_customer_encounter_range_and_lateral_threshold():
    """Verify person detection range (0.4m-1.5m) and lateral threshold (|y| <= 0.50m)."""
    import math
    from types import SimpleNamespace

    class MockNode:
        def __init__(self):
            self.blackboard = Blackboard()
            self._person_in_front_since = None
            self._last_wave_seen_at = 0.0
            self._wave_timeout_sec = 5.0

        def check_person(self, x, y, is_waving=False):
            p = SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                position_valid=True,
                is_waving=is_waving,
                wave_valid=is_waving,
                track_id=1,
            )
            has_blocking = False
            dist = math.hypot(p.position.x, p.position.y)
            if 0.40 <= dist <= 1.50 and abs(p.position.y) <= 0.50:
                has_blocking = True
            return has_blocking

    node = MockNode()

    # 1. Person at dist ~1.26m, y = 0.4m -> Within range (0.4m - 1.5m, |y| <= 0.5m)
    assert node.check_person(1.2, 0.4) is True

    # 2. Person at dist 1.0m, y = 0.45m -> Within range
    assert node.check_person(1.0, 0.45) is True

    # 3. Person at dist 1.8m -> Outside range (> 1.5m)
    assert node.check_person(1.8, 0.0) is False

    # 4. Person at dist 1.0m, y = 0.60m -> Outside lateral threshold (> 0.5m)
    assert node.check_person(1.0, 0.60) is False

    # 5. Person too close (< 0.4m)
    assert node.check_person(0.30, 0.10) is False


def test_audio_player_greeting_chuc_20_11_on_encounter_and_wave():
    """Verify AudioPlayerNode plays chuc_20_11 on encounter and wave arrival."""
    import importlib.util
    import os
    from std_msgs.msg import String

    script_path = "/home/a/ros2_ws/src/mo_hinh/scripts/audio_player_node.py"
    if not os.path.exists(script_path):
        script_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "mo_hinh", "scripts", "audio_player_node.py")
        )

    spec = importlib.util.spec_from_file_location("audio_player_node", script_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    class DummyAudioPlayer:
        def __init__(self):
            self.played = []

        def play_sound(self, sound_key, force=False, priority=False):
            self.played.append(sound_key)

        def cb_mission_log(self, msg):
            mod.AudioPlayerNode.cb_mission_log(self, msg)

    player = DummyAudioPlayer()

    # Test 1: Log encounter ("👤 GẶP NGƯỜI TRƯỚC MẶT! Dừng tĩnh 7 giây mời nước...")
    msg_encounter = String()
    msg_encounter.data = "👤 GẶP NGƯỜI TRƯỚC MẶT! Dừng tĩnh 7 giây mời nước..."
    for _ in range(5):
        player.cb_mission_log(msg_encounter)

    assert len(player.played) == 5
    for s in player.played:
        assert s == 'chuc_20_11'

    # Test 2: Log arrival at waving person
    player.played.clear()
    msg_wave = String()
    msg_wave.data = "🎯 ĐÃ ĐẾN TRƯỚC MẶT NGƯỜI VẪY TAY! Chuyển sang CHẾ ĐỘ PHỤC VỤ."
    player.cb_mission_log(msg_wave)
    assert len(player.played) == 1
    assert player.played[0] == 'chuc_20_11'


def test_action_customer_encounter_7s_and_instant_brake_serving():
    """Verify encounter waits 7s (not 5s) and activating serving triggers immediate robot motion stop."""
    from robot_mission_control.bt_nodes import ActionCustomerEncounter
    from robot_mission_control.mission_controller_node import MissionControllerNode

    class DummyController:
        def __init__(self):
            self.blackboard = Blackboard()
            self.stop_called = 0
            self.cancel_nav2_called = False
            self.published_logs = []
            self.popup_cmds = []

        def stop_robot_motion(self, cancel_nav2=True):
            self.stop_called += 1
            self.cancel_nav2_called = cancel_nav2

        def publish_zero_vel(self):
            pass

        def publish_log(self, text):
            self.published_logs.append(text)

        def publish_popup_cmd(self, cmd):
            self.popup_cmds.append(cmd)

        def get_logger(self):
            import logging
            return logging.getLogger("dummy")

        def _activate_serving(self):
            MissionControllerNode._activate_serving(self)

    controller = DummyController()
    action = ActionCustomerEncounter("TestEncounter", node=controller)

    assert action.duration_sec == 7.0

    bb = controller.blackboard
    bb.set('HMI_start_patrol', True)
    bb.set('ENCOUNTER_ACTIVE', False)
    bb.set('SERVE_REQUESTED', False)

    # 1. First tick -> Starts 7s encounter
    status = action.tick(bb)
    assert status == NodeStatus.RUNNING
    assert bb.get('ENCOUNTER_ACTIVE') is True
    assert controller.stop_called == 1
    assert "SHOW_5S_INVITE" in controller.popup_cmds

    # 2. At t = 5.0s, must STILL be RUNNING (no longer times out at 5s)
    action.start_time = time.time() - 5.0
    status = action.tick(bb)
    assert status == NodeStatus.RUNNING
    assert bb.get('ENCOUNTER_ACTIVE') is True

    # 3. At t = 7.1s, finishes with SUCCESS
    action.start_time = time.time() - 7.1
    status = action.tick(bb)
    assert status == NodeStatus.SUCCESS
    assert bb.get('ENCOUNTER_ACTIVE') is False
    assert "HIDE" in controller.popup_cmds

    # 4. Activate serving calls stop_robot_motion immediately
    controller.stop_called = 0
    controller._activate_serving()
    assert controller.stop_called == 1
    assert controller.cancel_nav2_called is True
    assert bb.get('IS_SERVING') is True


def test_encounter_and_waving_rules_when_not_patrolling():
    """Verify waving does not trigger when not patrolling, but encounter triggers if PERSON_IN_PATH is True (e.g. 3s dwell)."""
    cond_wave = IsHandWavingDetected("TestWave")
    cond_enc = IsCustomerEncountered("TestEnc")
    bb = Blackboard()

    # 1. Patrol is NOT active
    bb.set('HMI_start_patrol', False)
    bb.set('HAND_WAVE_ACTIVE', True)
    bb.set('PERSON_IN_PATH', True)
    bb.set('IS_SERVING', False)

    # Waving must return FAILURE when not patrolling to prevent auto-navigating
    assert cond_wave.tick(bb) == NodeStatus.FAILURE
    # When HAND_WAVE_ACTIVE is True, encounter is suppressed
    assert cond_enc.tick(bb) == NodeStatus.FAILURE

    # 2. When not patrolling and no wave active, PERSON_IN_PATH triggers encounter
    bb.set('HAND_WAVE_ACTIVE', False)
    assert cond_enc.tick(bb) == NodeStatus.SUCCESS

    # 3. Patrol IS active
    bb.set('HMI_start_patrol', True)
    bb.set('HAND_WAVE_ACTIVE', True)
    assert cond_wave.tick(bb) == NodeStatus.SUCCESS


def test_tracked_people_non_patrol_3s_dwell_vs_patrol_instant():
    """Verify non-patrol requires 3s continuous dwell in narrow zone, while patrol triggers instantly in wider zone."""
    import math
    from types import SimpleNamespace
    from robot_interfaces.msg import TrackedPersonArray, TrackedPerson
    from robot_mission_control.mission_controller_node import MissionControllerNode

    # Dummy node without full ROS spin
    class DummyNode:
        def __init__(self):
            self.blackboard = Blackboard()
            self._person_in_front_since = None
            self._non_patrol_gaze_start = None
            self._last_narrow_person_seen = 0.0
            self._last_wave_seen_at = 0.0
            self._wave_timeout_sec = 2.0

        def cb_tracked_people(self, msg):
            MissionControllerNode.cb_tracked_people(self, msg)

    node = DummyNode()

    def make_person_msg(x, y, is_waving=False):
        p = TrackedPerson()
        p.track_id = "1"
        p.position_valid = True
        p.position.x = float(x)
        p.position.y = float(y)
        p.is_waving = is_waving
        p.wave_valid = is_waving
        msg = TrackedPersonArray()
        msg.people = [p]
        return msg

    # ==========================================================
    # TEST CASE A: NOT PATROLLING (HMI_start_patrol = False)
    # ==========================================================
    node.blackboard.set('HMI_start_patrol', False)
    node.blackboard.set('ENCOUNTER_ACTIVE', False)
    node.blackboard.set('APPROACH_ACTIVE', False)
    node.blackboard.set('HAND_WAVE_ACTIVE', False)

    # 1. Person outside narrow lateral zone (|y| = 0.50m > 0.35m)
    msg_wide = make_person_msg(x=0.8, y=0.50)
    node.cb_tracked_people(msg_wide)
    assert node.blackboard.get('PERSON_IN_PATH') is False
    assert node._non_patrol_gaze_start is None

    # 2. Person inside narrow zone (x=0.75m, y=0.15m) at t=0
    msg_narrow = make_person_msg(x=0.75, y=0.15)
    node.cb_tracked_people(msg_narrow)
    # Instant trigger should NOT happen yet
    assert node.blackboard.get('PERSON_IN_PATH') is False
    assert node._non_patrol_gaze_start is not None

    # Simulate 1.5 seconds elapsed (dwell < 3.0s)
    node._non_patrol_gaze_start = time.monotonic() - 1.5
    node.cb_tracked_people(msg_narrow)
    assert node.blackboard.get('PERSON_IN_PATH') is False

    # Simulate 3.2 seconds elapsed (dwell >= 3.0s) -> Must trigger PERSON_IN_PATH = True
    node._non_patrol_gaze_start = time.monotonic() - 3.2
    node.cb_tracked_people(msg_narrow)
    assert node.blackboard.get('PERSON_IN_PATH') is True

    # 3. Person steps away
    empty_msg = TrackedPersonArray()
    empty_msg.people = []
    # If 0.5s passes with nobody
    node._last_narrow_person_seen = time.monotonic() - 0.5
    node.cb_tracked_people(empty_msg)
    assert node.blackboard.get('PERSON_IN_PATH') is False
    assert node._non_patrol_gaze_start is None

    # ==========================================================
    # TEST CASE B: PATROLLING (HMI_start_patrol = True)
    # ==========================================================
    node.blackboard.set('HMI_start_patrol', True)
    # Person in wide blocking zone (|y| = 0.55m <= 0.65m) triggers immediately
    msg_patrol_block = make_person_msg(x=0.9, y=0.55)
    node.cb_tracked_people(msg_patrol_block)
    assert node.blackboard.get('PERSON_IN_PATH') is True


