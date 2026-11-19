"""Unit tests for HomePage MissionTimelineWidget and Event Branching."""
from __future__ import annotations

import pytest
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.home_page import HomePage, MissionTimelineWidget


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    return app


def test_mission_timeline_initialization(qapp):
    widget = MissionTimelineWidget()
    assert widget is not None
    assert widget.objectName() == "missionTimelineCard"
    assert widget.event_branch_box.isHidden()


def test_mission_timeline_route_rendering(qapp):
    widget = MissionTimelineWidget()
    route = ["khoihanh", "diem1", "diem2", "tram_cho"]
    widget.set_patrol_route(route)
    assert widget._patrol_route == route

    # Set moving to diem1
    widget.set_navigation_status({
        "status": "MOVING",
        "current_goal_name": "diem1",
        "distance_remaining": 4.5,
    })

    assert widget._current_goal_name == "diem1"
    assert widget._distance_remaining == 4.5
    assert "diem1" in widget.lbl_goal_info.text()
    assert "4.5m" in widget.lbl_dist_info.text()

    # khoihanh should be completed (chipCompleted), diem1 active (chipActive), diem2 pending
    chips = widget.timeline_container.findChildren(object, name="chipCompleted")
    active_chips = widget.timeline_container.findChildren(object, name="chipActive")
    pending_chips = widget.timeline_container.findChildren(object, name="chipPending")

    assert len(chips) >= 1  # khoihanh is completed
    assert len(active_chips) == 1  # diem1 is active
    assert len(pending_chips) >= 1  # diem2, tram_cho are pending


def test_mission_timeline_event_branching_and_graying(qapp):
    widget = MissionTimelineWidget()
    widget.set_patrol_route(["khoihanh", "diem1", "diem2"])
    widget.set_navigation_status({
        "status": "MOVING",
        "current_goal_name": "diem1",
        "distance_remaining": 3.0,
    })

    # Initially not branching
    assert not widget._is_branching_event
    assert widget.event_branch_box.isHidden()

    # Trigger 5s invite event
    widget.trigger_event_branch(title="Mời khách 5s", duration_sec=5, description="Đang chào mời khách...")

    assert widget._is_branching_event
    assert not widget.event_branch_box.isHidden()
    assert "Mời khách 5s" in widget.lbl_event_title.text()
    assert "5s" in widget.lbl_event_countdown.text()

    # The active chip should now be PAUSED / GRAYED OUT
    paused_chips = widget.timeline_container.findChildren(object, name="chipPaused")
    active_chips = widget.timeline_container.findChildren(object, name="chipActive")
    assert len(paused_chips) == 1
    assert len(active_chips) == 0

    # Clear event
    widget.clear_event_branch()
    assert not widget._is_branching_event
    assert widget.event_branch_box.isHidden()

    # Active chip restored
    active_chips_restored = widget.timeline_container.findChildren(object, name="chipActive")
    assert len(active_chips_restored) == 1


def test_home_page_integration(qapp):
    home = HomePage()
    assert hasattr(home, "mission_timeline")
    assert home.mission_timeline is not None

    # Test forwarding methods
    home.set_patrol_route(["khoihanh", "diem1", "tram_cho"])
    assert home.mission_timeline._patrol_route == ["khoihanh", "diem1", "tram_cho"]

    home.set_navigation_status({
        "status": "MOVING",
        "current_goal_name": "diem1",
        "distance_remaining": 2.5,
    })
    assert home.mission_timeline._current_goal_name == "diem1"

    home.trigger_event_branch("Mời người", 5)
    assert home.mission_timeline._is_branching_event

    home.clear_event_branch()
    assert not home.mission_timeline._is_branching_event
