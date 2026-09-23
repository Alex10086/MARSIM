"""Tests for goal setpoint selection (latched vs streaming semantics).

RViz 2D Goal Pose publishes the goal exactly ONCE per click (latched/one-shot
semantics), while a streaming planner republishes the setpoint every cycle.
The original code assumed streaming: after `goal_timeout` seconds it discarded
the goal and held the current position. Against a one-shot goal this aborted
the flight mid-way (e.g. a goal at x=4.0 stopped at x=2.1).

`goal_is_active` encodes the correct rule for both cases.
"""
from quad_pid.goal import goal_is_active


def test_no_goal_is_inactive():
    assert goal_is_active(has_goal=False, goal_age=0.0,
                          goal_timeout=1.0, latched=True) is False


def test_latched_goal_stays_active_when_old():
    # The bug: an old one-shot goal must still be tracked.
    assert goal_is_active(has_goal=True, goal_age=5.0,
                          goal_timeout=1.0, latched=True) is True


def test_latched_goal_is_active_when_fresh():
    assert goal_is_active(has_goal=True, goal_age=0.1,
                          goal_timeout=1.0, latched=True) is True


def test_streaming_goal_expires_when_old():
    assert goal_is_active(has_goal=True, goal_age=5.0,
                          goal_timeout=1.0, latched=False) is False


def test_streaming_goal_active_when_fresh():
    assert goal_is_active(has_goal=True, goal_age=0.1,
                          goal_timeout=1.0, latched=False) is True