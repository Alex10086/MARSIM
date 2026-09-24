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


# ── SetpointResolver：模式状态机 ────────────────────────────────────────
import numpy as np
from quad_pid.modes import SetpointResolver

CFG = {
    'twist_max_vx': 1.5, 'twist_max_vy': 1.5, 'twist_max_wz': 0.8,
    'twist_max_vz': 1.0,
    'twist_target_height': -1.0, 'twist_follow_z': False,
    'cmd_vel_timeout': 0.5,
}

def _r(leash=5.0):
    return SetpointResolver(leash=leash)

def _upd(r, *, mode='auto', dt=0.1, pos=(0., 0., 5.), yaw=0.0,
         goal_active=False, goal_stamp=0.0, goal_xyz=None, goal_yaw=None,
         twist_fresh=False, twist_stamp=0.0, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
    return r.update(mode=mode, dt=dt, cur_pos=np.array(pos), cur_yaw=yaw,
                    goal_active=goal_active, goal_stamp=goal_stamp,
                    goal_xyz=goal_xyz, goal_yaw=goal_yaw,
                    twist_fresh=twist_fresh, twist_stamp=twist_stamp,
                    twist_vx=vx, twist_vy=vy, twist_vz=vz, twist_wz=wz, cfg=CFG)


def test_no_command_ever_holds_current_pose():
    # 与今天的行为逐位一致：无目标无 Twist → 保持当前位姿
    r = _r()
    p, y, v, s = _upd(r, pos=(3.0, 4.0, 5.0), yaw=1.1)
    assert s == HOLD
    assert p == pytest.approx(np.array([3.0, 4.0, 5.0]))
    assert y == pytest.approx(1.1)
    assert v == pytest.approx(np.zeros(3))


def test_goal_mode_returns_goal_and_zero_velocity_feedforward():
    r = _r()
    p, y, v, s = _upd(r, mode=POSITION, goal_active=True, goal_stamp=1.0,
                      goal_xyz=(7.0, 8.0, 3.0), goal_yaw=0.4)
    assert s == POSITION
    assert p == pytest.approx(np.array([7.0, 8.0, 3.0]))
    assert y == pytest.approx(0.4)
    assert v == pytest.approx(np.zeros(3))     # Review Focus #9


def test_first_velocity_tick_is_bumpless():
    # Review Focus #3: 首个 tick 的设定点必须 ≈ 当前位姿，不能跳
    r = _r()
    p, y, v, s = _upd(r, mode=VELOCITY, pos=(3.0, 4.0, 5.0), yaw=0.7,
                      twist_fresh=True, twist_stamp=1.0, vx=1.0, dt=0.0)
    assert s == VELOCITY
    assert p == pytest.approx(np.array([3.0, 4.0, 5.0]))
    assert y == pytest.approx(0.7)


def test_velocity_mode_advances_setpoint_and_feeds_world_velocity():
    r = _r()
    _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.0, vx=1.0)      # latch
    p, y, v, s = _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.1, vx=1.0)
    assert p[0] == pytest.approx(0.1, abs=1e-9)
    assert p[1] == pytest.approx(0.0, abs=1e-9)
    assert v == pytest.approx(np.array([1.0, 0.0, 0.0]))          # 世界系前馈


def test_velocity_feedforward_rotates_with_yaw():
    # Review Focus #1
    r = _r()
    yaw = math.radians(-54.0)
    _upd(r, mode=VELOCITY, yaw=yaw, twist_fresh=True, dt=0.0, vx=1.0)
    p, y, v, s = _upd(r, mode=VELOCITY, yaw=yaw, twist_fresh=True, dt=0.1, vx=1.0)
    assert v[0] == pytest.approx(math.cos(yaw), abs=1e-9)
    assert v[1] == pytest.approx(math.sin(yaw), abs=1e-9)
    assert p[0] == pytest.approx(0.1 * math.cos(yaw), abs=1e-9)
    assert p[1] == pytest.approx(0.1 * math.sin(yaw), abs=1e-9)


def test_height_is_latched_on_entering_velocity_mode():
    # D4
    r = _r()
    _upd(r, mode=VELOCITY, pos=(0., 0., 7.3), dt=0.0, twist_fresh=True)
    p, *_ = _upd(r, mode=VELOCITY, pos=(0., 0., 7.3), dt=0.1, twist_fresh=True, vx=1.0)
    assert p[2] == pytest.approx(7.3)


def test_twist_target_height_overrides_latch():
    cfg = dict(CFG, twist_target_height=12.0)
    r = _r()
    p, *_ = r.update(mode=VELOCITY, dt=0.1, cur_pos=np.array([0., 0., 7.3]),
                     cur_yaw=0.0, goal_active=False, goal_stamp=0.0,
                     goal_xyz=None, goal_yaw=None, twist_fresh=True,
                     twist_stamp=1.0, twist_vx=1.0, twist_vy=0.0,
                     twist_vz=0.0, twist_wz=0.0, cfg=cfg)
    assert p[2] == pytest.approx(12.0)


def test_twist_follow_z_integrates_linear_z():
    cfg = dict(CFG, twist_follow_z=True)
    r = _r()
    r.update(mode=VELOCITY, dt=0.0, cur_pos=np.array([0., 0., 5.0]), cur_yaw=0.0,
             goal_active=False, goal_stamp=0.0, goal_xyz=None, goal_yaw=None,
             twist_fresh=True, twist_stamp=1.0, twist_vx=0.0, twist_vy=0.0,
             twist_vz=0.5, twist_wz=0.0, cfg=cfg)
    p, *_ = r.update(mode=VELOCITY, dt=0.1, cur_pos=np.array([0., 0., 5.0]),
                     cur_yaw=0.0, goal_active=False, goal_stamp=0.0,
                     goal_xyz=None, goal_yaw=None, twist_fresh=True,
                     twist_stamp=1.0, twist_vx=0.0, twist_vy=0.0,
                     twist_vz=0.5, twist_wz=0.0, cfg=cfg)
    assert p[2] == pytest.approx(5.05, abs=1e-9)


def test_stale_twist_freezes_the_setpoint_never_keeps_integrating():
    # Review Focus #2 —— 最关键的安全属性
    r = _r()
    for _ in range(5):
        _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.1, vx=1.0)
    frozen, _, _, s = _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.1, vx=1.0)
    for _ in range(50):
        stalled, _, vel, s2 = _upd(r, mode=VELOCITY, twist_fresh=False, dt=0.1, vx=1.0)
        assert s2 == HOLD
        assert stalled == pytest.approx(frozen)      # 冻结，不前进
        assert vel == pytest.approx(np.zeros(3))     # 前馈归零 → 主动刹车


def test_velocity_mode_without_any_twist_holds_current_pose():
    # control_mode="velocity" 但 Nav2 还没起来 → 必须保持当前位姿而不是乱飞
    r = _r()
    p, y, v, s = _upd(r, mode=VELOCITY, pos=(1.0, 2.0, 3.0), yaw=0.2, twist_fresh=False)
    assert s == HOLD
    assert p == pytest.approx(np.array([1.0, 2.0, 3.0]))


def test_mode_switch_back_and_forth_is_bumpless():
    r = _r()
    for _ in range(5):
        _upd(r, mode=VELOCITY, pos=(0., 0., 5.), twist_fresh=True, dt=0.1, vx=1.0)
    _upd(r, mode=POSITION, pos=(0.5, 0., 5.), goal_active=True, goal_stamp=9.0,
         goal_xyz=(20.0, 0.0, 5.0), goal_yaw=0.0, dt=0.1)
    # 重新进入 velocity：必须重新锁存到当前位置，而不是复用旧 ref
    p, _, _, s = _upd(r, mode=VELOCITY, pos=(0.5, 0., 5.), twist_fresh=True,
                      twist_stamp=20.0, dt=0.0, vx=1.0)
    assert s == VELOCITY
    assert p == pytest.approx(np.array([0.5, 0.0, 5.0]))


def test_auto_mode_twist_wins_while_nav2_streams():
    r = _r()
    p, _, _, s = _upd(r, mode='auto', goal_active=True, goal_stamp=1.0,
                      goal_xyz=(50.0, 50.0, 5.0), goal_yaw=0.0,
                      twist_fresh=True, twist_stamp=9.0, dt=0.1, vx=1.0)
    assert s == VELOCITY


def test_leash_holds_setpoint_when_vehicle_cannot_move():
    # Review Focus #6
    r = _r(leash=2.0)
    for _ in range(500):
        p, *_ = _upd(r, mode=VELOCITY, pos=(0., 0., 5.), dt=0.1, vx=1.5, twist_fresh=True)
    assert math.hypot(p[0], p[1]) == pytest.approx(2.0, abs=1e-6)


def test_yaw_rate_integrates_and_wraps():
    # NOTE: use a rate inside twist_max_wz (=0.8) or clamp_twist will saturate it
    r = _r()
    _upd(r, mode=VELOCITY, yaw=math.pi - 0.01, dt=0.0, twist_fresh=True, wz=0.5)
    _, y, _, _ = _upd(r, mode=VELOCITY, yaw=math.pi - 0.01, dt=0.05,
                      twist_fresh=True, wz=0.5)
    # pi - 0.01 + 0.5*0.05 = pi + 0.015 -> wrapped to -pi + 0.015
    assert abs(y) <= math.pi
    assert y == pytest.approx(-math.pi + 0.015, abs=1e-9)


def test_yaw_rate_is_clamped():
    r = _r()
    _upd(r, mode=VELOCITY, dt=0.0, twist_fresh=True, wz=0.0)
    _, y, _, _ = _upd(r, mode=VELOCITY, dt=1.0, twist_fresh=True, wz=99.0)
    assert y == pytest.approx(0.8, abs=1e-9)      # twist_max_wz


def test_hold_after_velocity_session_returns_frozen_reference():
    r = _r()
    for _ in range(3):
        _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.1, vx=1.0)
    p, _, _, s = _upd(r, mode=VELOCITY, twist_fresh=False, dt=0.1, pos=(0.0, 0.0, 5.0))
    assert s == HOLD
    assert p[0] > 0.0     # 冻结在 leash 内的参考点（而非回到原点）


def test_hold_does_not_travel_to_a_stale_reference_after_a_mode_round_trip():
    # Finding from Task 6 Step 3 (hot-switch verification). After a velocity
    # session, leaving velocity mode and later forcing it back on with no fresh
    # Twist must HOLD WHERE THE VEHICLE IS, not fly back to the reference that
    # was frozen during the old session. That reference is only leashed to the
    # vehicle at the time it was written, so after a long excursion in position
    # mode the drone would otherwise fly arbitrarily far back to it.
    r = _r()
    for _ in range(5):
        _upd(r, mode=VELOCITY, twist_fresh=True, dt=0.1, vx=1.0)   # ref -> 0.5
    _upd(r, mode=POSITION, pos=(50., 0., 5.), goal_active=True, goal_stamp=9.0,
         goal_xyz=(50.0, 0.0, 5.0), goal_yaw=0.0, dt=0.1)
    p, y, v, s = _upd(r, mode=VELOCITY, pos=(50., 0., 5.), twist_fresh=False)
    assert s == HOLD
    assert p == pytest.approx(np.array([50.0, 0.0, 5.0]))
    assert v == pytest.approx(np.zeros(3))


# ── 最终评审发现（code review findings）──────────────────────────────
from quad_pid.modes import clamp_dt, MIN_HEIGHT, MAX_HEIGHT


def test_clamp_dt_bounds_a_suspended_process():
    # Review Focus #8: a process suspended by the debugger/scheduler must not
    # advance the setpoint integrator by seconds' worth in one tick.
    assert clamp_dt(5.0) == pytest.approx(0.1)
    assert clamp_dt(-1.0) == pytest.approx(0.0)
    assert clamp_dt(0.02) == pytest.approx(0.02)
    assert clamp_dt(float('nan')) == pytest.approx(0.0)
    assert clamp_dt(1e308) == pytest.approx(0.1)


def test_velocity_feedforward_is_exactly_zero_for_position_and_hold():
    # Review Focus #9: the node computes e_vel = vel_des - vel, so position and
    # hold must return EXACTLY zeros. Anything else perturbs the pre-Twist
    # behaviour that was verified bit-identical.
    r = _r()
    _, _, v_pos, s_pos = _upd(r, mode=POSITION, goal_active=True, goal_stamp=1.0,
                              goal_xyz=(1.0, 1.0, 1.0), goal_yaw=0.0)
    assert s_pos == POSITION
    assert not v_pos.any()

    r2 = _r()
    _, _, v_hold, s_hold = _upd(r2, pos=(2.0, 3.0, 4.0))
    assert s_hold == HOLD
    assert not v_hold.any()


def test_non_finite_odom_never_poisons_the_reference():
    # Code review finding #1. A single corrupt odom sample must not be consumed:
    # latching a NaN into ref_xy is ABSORBING (advance_setpoint's `d > leash` is
    # False for NaN, so it is rewritten every tick), and the downstream clamps
    # turn NaN into a FULL-SCALE command, so the end-of-pipeline NaN guard never
    # trips. Without the guard the drone flies max acceleration until odom
    # recovers or the source changes.
    r = _r()
    good, _, _, _ = _upd(r, mode=VELOCITY, pos=(1.0, 2.0, 5.0), twist_fresh=True,
                         dt=0.1, vx=1.0)
    bad = np.array([float('nan'), float('inf'), float('-inf')])
    p, y, v, s = _upd(r, mode=VELOCITY, pos=tuple(bad), twist_fresh=True,
                      dt=0.1, vx=1.0)
    assert s == HOLD
    assert np.all(np.isfinite(p))
    assert p == pytest.approx(good)          # held the last good command
    assert not v.any()
    # ... and the reference is NOT poisoned: a good sample resumes normally.
    p2, _, _, s2 = _upd(r, mode=VELOCITY, pos=(1.1, 2.0, 5.0), twist_fresh=True,
                        dt=0.1, vx=1.0)
    assert s2 == VELOCITY
    assert np.all(np.isfinite(p2))


def test_non_finite_odom_before_any_good_tick_is_safe():
    r = _r()
    p, y, v, s = _upd(r, mode=VELOCITY, pos=(float('nan'),) * 3, twist_fresh=True)
    assert s == HOLD
    assert np.all(np.isfinite(p)) and np.all(np.isfinite(v))


def test_twist_follow_z_is_rate_limited():
    # Code review finding #2: only `isfinite` was checked, so a FINITE absurd
    # value (1e308) overflowed ref_z to inf within a few ticks; pos_des[2]=inf
    # then made the node's S2 carrot compute inf * 0.0 = NaN.
    cfg = dict(CFG, twist_follow_z=True, twist_max_vz=0.5)
    r = SetpointResolver(5.0)
    kw = dict(mode=VELOCITY, cur_pos=np.array([0., 0., 5.0]), cur_yaw=0.0,
              goal_active=False, goal_stamp=0.0, goal_xyz=None, goal_yaw=None,
              twist_fresh=True, twist_stamp=1.0, twist_vx=0.0, twist_vy=0.0,
              twist_wz=0.0, cfg=cfg)
    r.update(dt=0.0, twist_vz=0.0, **kw)
    p, *_ = r.update(dt=1.0, twist_vz=1e308, **kw)
    assert np.isfinite(p[2])
    assert p[2] == pytest.approx(5.0 + 0.5 * 1.0, abs=1e-9)


def test_ref_z_is_banded():
    cfg = dict(CFG, twist_follow_z=True, twist_max_vz=5.0)
    r = SetpointResolver(5.0)
    kw = dict(mode=VELOCITY, cur_pos=np.array([0., 0., 0.5]), cur_yaw=0.0,
              goal_active=False, goal_stamp=0.0, goal_xyz=None, goal_yaw=None,
              twist_fresh=True, twist_stamp=1.0, twist_vx=0.0, twist_vy=0.0,
              twist_wz=0.0, cfg=cfg)
    for _ in range(20):
        p, *_ = r.update(dt=0.1, twist_vz=-5.0, **kw)
    assert p[2] == pytest.approx(MIN_HEIGHT)

    r2 = SetpointResolver(5.0)
    for _ in range(200):
        p2, *_ = r2.update(dt=0.1, twist_vz=5.0, **kw)
    assert p2[2] == pytest.approx(MAX_HEIGHT)
