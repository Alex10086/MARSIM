import math
import pytest

from quad_pid.geometry import wrap_to_pi
from quad_pid.modes import (POSITION, VELOCITY, HOLD,
                            clamp_twist, body_to_world_velocity,
                            advance_yaw, advance_setpoint, select_source)


# ── wrap_to_pi ────────────────────────────────────────────────────────
def test_wrap_to_pi_leaves_in_range_untouched():
    for a in (0.0, 1.0, -1.0, math.pi - 1e-9):
        assert wrap_to_pi(a) == pytest.approx(a, abs=1e-12)

def test_wrap_to_pi_wraps_both_directions():
    assert wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-9)
    assert wrap_to_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, abs=1e-9)
    assert wrap_to_pi(3 * math.pi) == pytest.approx(math.pi, abs=1e-9)


# ── clamp_twist ───────────────────────────────────────────────────────
def test_clamp_twist_passes_through_within_limits():
    assert clamp_twist(1.0, -0.5, 0.3, 1.5, 1.5, 0.8) == (1.0, -0.5, 0.3)

def test_clamp_twist_saturates_each_axis_independently():
    vx, vy, wz = clamp_twist(9.0, -9.0, 9.0, 1.5, 1.5, 0.8)
    assert (vx, vy, wz) == (1.5, -1.5, 0.8)

def test_clamp_twist_sanitises_nan_and_inf():
    vx, vy, wz = clamp_twist(float('nan'), float('inf'), float('-inf'), 1.5, 1.5, 0.8)
    assert vx == 0.0
    assert vy == 1.5
    assert wz == -0.8


# ── body_to_world_velocity ────────────────────────────────────────────
def test_body_to_world_at_zero_yaw_is_identity():
    assert body_to_world_velocity(1.0, 0.0, 0.0) == pytest.approx((1.0, 0.0))

def test_body_to_world_at_yaw_90deg_forward_becomes_plus_y():
    # Nav2 的 Twist 是机体帧：yaw=+90° 时「向前」在世界系是 +y
    wx, wy = body_to_world_velocity(1.0, 0.0, math.pi / 2)
    assert wx == pytest.approx(0.0, abs=1e-9)
    assert wy == pytest.approx(1.0, abs=1e-9)

def test_body_to_world_at_negative_54deg_matches_hand_computed():
    # 绕圈 bug 的回归：yaw=-54° 时 vx=1 必须走世界 -54° 方向
    yaw = math.radians(-54.0)
    wx, wy = body_to_world_velocity(1.0, 0.0, yaw)
    assert wx == pytest.approx(math.cos(yaw), abs=1e-12)
    assert wy == pytest.approx(math.sin(yaw), abs=1e-12)

def test_body_to_world_lateral_at_yaw_90deg_becomes_minus_x():
    wx, wy = body_to_world_velocity(0.0, 1.0, math.pi / 2)
    assert wx == pytest.approx(-1.0, abs=1e-9)
    assert wy == pytest.approx(0.0, abs=1e-9)

def test_body_to_world_preserves_speed():
    wx, wy = body_to_world_velocity(1.2, -0.7, -0.94)
    assert math.hypot(wx, wy) == pytest.approx(math.hypot(1.2, -0.7), abs=1e-12)


# ── advance_yaw ───────────────────────────────────────────────────────
def test_advance_yaw_integrates_rate():
    assert advance_yaw(0.0, 0.5, 0.1) == pytest.approx(0.05, abs=1e-12)

def test_advance_yaw_wraps_continuously_across_pi():
    # 自转穿过 ±π 不能突跳
    y = advance_yaw(math.pi - 1e-3, 1.0, 0.01)
    assert y == pytest.approx(-math.pi + 1e-2 - 1e-3, abs=1e-9)
    assert abs(y) <= math.pi


# ── advance_setpoint ──────────────────────────────────────────────────
def test_advance_setpoint_free_runs_when_within_leash():
    out = advance_setpoint((0.0, 0.0), (1.0, 0.0), 0.1, 5.0, (0.0, 0.0))
    assert out == pytest.approx((0.1, 0.0))

def test_advance_setpoint_clamps_to_leash_when_blocked():
    # 无人机被钉住：设定点必须被牵引绳拉住，绝不跑远（防积分饱和）
    ref = (0.0, 0.0)
    cur = (0.0, 0.0)
    for _ in range(1000):
        ref = advance_setpoint(ref, (5.0, 0.0), 0.1, 2.0, cur)
    d = math.hypot(ref[0] - cur[0], ref[1] - cur[1])
    assert d == pytest.approx(2.0, abs=1e-9)

def test_advance_setpoint_clamps_along_correct_direction():
    out = advance_setpoint((10.0, 10.0), (0.0, 0.0), 0.1, 1.0, (0.0, 0.0))
    assert math.hypot(out[0], out[1]) == pytest.approx(1.0, abs=1e-9)
    assert out[0] == pytest.approx(out[1], abs=1e-9)   # 方向保持 45°

def test_advance_setpoint_zero_dt_is_noop():
    assert advance_setpoint((0.3, -0.2), (1.0, 1.0), 0.0, 5.0, (0.0, 0.0)) \
        == pytest.approx((0.3, -0.2))


# ── select_source ─────────────────────────────────────────────────────
def test_select_source_position_mode_requires_usable_goal():
    assert select_source(POSITION, True, 1.0, True, 2.0) == POSITION
    assert select_source(POSITION, False, 1.0, True, 2.0) == HOLD

def test_select_source_velocity_mode_requires_fresh_twist():
    assert select_source(VELOCITY, True, 1.0, True, 2.0) == VELOCITY
    assert select_source(VELOCITY, True, 1.0, False, 2.0) == HOLD

def test_select_source_auto_picks_newer_stamp():
    assert select_source('auto', True, 5.0, True, 9.0) == VELOCITY
    assert select_source('auto', True, 9.0, True, 5.0) == POSITION

def test_select_source_auto_holds_when_neither_usable():
    assert select_source('auto', False, 0.0, False, 0.0) == HOLD

def test_select_source_auto_falls_back_to_the_only_usable_source():
    assert select_source('auto', True, 9.0, False, 0.0) == POSITION
    assert select_source('auto', False, 0.0, True, 1.0) == VELOCITY

def test_select_source_forced_mode_ignores_the_other_source():
    # velocity 档下，哪怕位置目标更新，也不许位置源抢走控制权
    assert select_source(VELOCITY, True, 99.0, True, 1.0) == VELOCITY
