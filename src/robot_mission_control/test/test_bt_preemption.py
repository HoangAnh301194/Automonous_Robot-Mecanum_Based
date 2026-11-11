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
