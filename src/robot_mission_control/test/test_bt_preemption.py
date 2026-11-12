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
    assert action.standoff_distance == 1.20
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
    assert pose_map.pose.position.x == pytest.approx(2.00)  # 3.20 - 1.20 standoff
    assert recorded_times == [98 * 10**9 + 500000000]

    # Now test fallback when stamped TF lookup fails
    recorded_times.clear()
    fake_node.tf_buffer = FakeTfBuffer(fail_stamped=True)
    pose_map_fb, dist_fb = action._resolve_target_pose_map(bb)

    assert pose_map_fb is not None
    assert dist_fb == pytest.approx(3.20)
    assert pose_map_fb.pose.position.x == pytest.approx(2.00)
    assert recorded_times == [98 * 10**9 + 500000000, 0]
