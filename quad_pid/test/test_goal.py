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


# ── Goal sanity guards ────────────────────────────────────────────────
# RViz's 2D Goal Pose is a *2D* tool: it has no z, so it always publishes
# position.z == 0, and clicking near the horizon projects onto the z=0 plane
# kilometres away.  Blindly tracking such a goal makes the drone dive to the
# ground and then fly off toward a nonsense point.  The guards below sanitize
# the raw goal before it is used as a setpoint.

from quad_pid.goal import sanitize_goal


def test_goal_z_zero_keeps_current_height():
    # z=0 from a 2D tool means "height unspecified" -> hold current height.
    x, y, z, ok = sanitize_goal(2.0, 3.0, 0.0, 0.0, 0.0, 1.5, max_goal_dist=50.0)
    assert ok is True
    assert (x, y, z) == (2.0, 3.0, 1.5)


def test_goal_small_positive_z_is_honoured():
    x, y, z, ok = sanitize_goal(2.0, 3.0, 2.5, 0.0, 0.0, 1.5, max_goal_dist=50.0)
    assert ok is True
    assert z == 2.5


def test_goal_negative_z_keeps_current_height():
    x, y, z, ok = sanitize_goal(1.0, 1.0, -3.0, 0.0, 0.0, 1.0, max_goal_dist=50.0)
    assert ok is True
    assert z == 1.0


def test_goal_beyond_max_dist_from_vehicle_is_rejected():
    # A click near the horizon produces a goal kilometres away.
    x, y, z, ok = sanitize_goal(1000.0, 1000.0, 0.0, 0.0, 0.0, 1.0, max_goal_dist=50.0)
    assert ok is False


def test_goal_within_max_dist_from_vehicle_is_accepted():
    x, y, z, ok = sanitize_goal(10.0, 0.0, 2.0, 0.0, 0.0, 1.0, max_goal_dist=50.0)
    assert ok is True


def test_horizon_click_relative_to_a_moved_vehicle_is_rejected():
    # Vehicle already far from origin: distance must be vehicle-relative.
    x, y, z, ok = sanitize_goal(3100.0, 3100.0, 0.0, 3000.0, 3000.0, 1.0,
                                max_goal_dist=50.0)
    assert ok is False


def test_nan_goal_is_rejected():
    x, y, z, ok = sanitize_goal(float('nan'), 0.0, 1.0, 0.0, 0.0, 1.0,
                                max_goal_dist=50.0)
    assert ok is False
