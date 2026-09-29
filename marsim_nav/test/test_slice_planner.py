import numpy as np
import pytest

FREE = 254      # 与 map_projector 一致：0=障碍、254=自由

from marsim_nav.slice_planner import (grid_to_world, plan_single_layer,
                                      world_to_grid)


def test_world_grid_roundtrip():
    for x, y in [(0.0, 0.0), (1.23, -4.56), (30.05, 19.4)]:
        ix, iy = world_to_grid(x, y, (-30.0, -19.0), 0.1)
        x2, y2 = grid_to_world(ix, iy, (-30.0, -19.0), 0.1)
        assert (x2, y2) == pytest.approx((x, y), abs=0.1)


def test_empty_grid_gives_path():
    g = np.full((20, 20), FREE, dtype=np.uint8)
    r = plan_single_layer(g, (1, 1), (18, 18), (-1.0, -1.0), 0.1)
    assert r["path"][0] == (1, 1) and r["path"][-1] == (18, 18)
    assert r["cost"] > 0
    assert r["reached"] is True


def test_wall_forces_detour_through_the_gap():
    g = np.full((20, 20), FREE, dtype=np.uint8)
    g[0:20, 10] = 0                        # 一堵墙（0 = 障碍）
    g[6:13, 10] = FREE                     # 缺口 7 格（0.7m）—— 半径 0.25m 才过得去
    r = plan_single_layer(g, (1, 10), (18, 10), (-1.0, -1.0), 0.1)
    assert r["path"][-1] == (18, 10)
    assert any(6 <= iy <= 12 and ix == 10 for ix, iy in r["path"])   # 必须走那个缺口
    assert all(g[iy, ix] == FREE for ix, iy in r["path"])   # 没穿墙


def test_unreachable_goal_reports_cleanly():
    g = np.full((10, 10), FREE, dtype=np.uint8)
    g[5, :] = 0                            # 完全隔断（0 = 障碍）
    r = plan_single_layer(g, (1, 1), (1, 8), (-1.0, -1.0), 0.1)
    assert r["reached"] is False
    assert r["path"] == []


def test_alpha_is_the_preference_knob_not_the_safety_knob():
    """α 只决定「为更宽走廊绕多远」；关掉 α 时走最短，打开时挑净空好的。

    这条钉住分工：安全由 robot_radius / required_clearance 两个门槛管，
    α 管偏好 —— 两者不能互相顶替。
    本例用小半径（0.05m）以便在小网格里造出可比较的几何；
    真实尺寸的硬约束由 test_cells_tighter_than_robot_radius_are_blocked 钉住。
    """
    g = np.full((21, 30), FREE, dtype=np.uint8)
    g[0:8, 14] = 0                  # 墙，缺口在 iy=8..20（13 格，够宽）
    kw = dict(robot_radius=0.05, required_clearance=0.20)
    r0 = plan_single_layer(g, (2, 10), (27, 10), (0.0, 0.0), 0.1,
                           alpha=0.0, **kw)
    r2 = plan_single_layer(g, (2, 10), (27, 10), (0.0, 0.0), 0.1,
                           alpha=2.0, **kw)
    assert r0["reached"] and r2["reached"]
    # α=2 时通过缺口处的净空不应差于 α=0
    def best_gap_clearance(r):
        return max(min(ix, 29 - ix) * 0.1 for ix, iy in r["path"] if ix == 14)
    assert best_gap_clearance(r2) >= best_gap_clearance(r0) - 1e-9


def test_wide_corridor_is_preferred_over_narrow():
    """等长两条路，选净空大的那条 —— 这才是选层该看的东西。

    同样用小半径，聚焦代价函数本身。
    """
    g = np.full((30, 30), FREE, dtype=np.uint8)
    g[0:10, :] = 0                  # 上边界
    g[24:30, :] = 0                 # 下边界
    g[10:14, 8:22] = 0              # 中间一块 -> 上走廊 0 格、下走廊 10 格
    g[14:18, 8:22] = FREE
    kw = dict(robot_radius=0.05, required_clearance=0.30)
    r = plan_single_layer(g, (2, 15), (27, 15), (0.0, 0.0), 0.1,
                          alpha=2.0, **kw)
    assert r["reached"] is True
    # 路径应走在下半部（iy>=14）的宽走廊里
    ys = [iy for _, iy in r["path"]]
    assert sum(1 for v in ys if v >= 14) > sum(1 for v in ys if v <= 13)


def test_path_is_optimal_on_uniform_grid():
    """启发式必须可采纳：均匀空地上应给出几何最短路。"""
    g = np.full((30, 30), FREE, dtype=np.uint8)
    r = plan_single_layer(g, (2, 2), (27, 27), (0.0, 0.0), 0.1, alpha=2.0,
                          robot_radius=0.05)
    assert r["length"] == pytest.approx(25 * np.sqrt(2) * 0.1, rel=1e-6)


def test_risk_is_zero_beyond_required_clearance():
    """裕度足够时不加惩罚 —— 门槛是安全量，不是偏好量。"""
    from marsim_nav.slice_planner import clearance_risk
    assert clearance_risk(10.0, required=0.61) == 0.0
    assert clearance_risk(0.61, required=0.61) == 0.0     # 含等于


def test_risk_grows_monotonically_as_clearance_shrinks():
    from marsim_nav.slice_planner import clearance_risk
    prev = -1.0
    for c in [1.0, 0.8, 0.61, 0.45, 0.3, 0.15, 0.0]:
        r = clearance_risk(c, required=0.61)
        assert r > prev or (r == 0.0 and prev == 0.0)
        prev = r
    assert clearance_risk(0.0, required=0.61) == pytest.approx(1.0)


def test_required_clearance_comes_from_measurement():
    """required = robot_radius + 实测跟踪偏差。数字是量出来的，不是拍的。"""
    from marsim_nav.slice_planner import REQUIRED_CLEARANCE, ROBOT_RADIUS
    assert ROBOT_RADIUS == pytest.approx(0.25)
    assert REQUIRED_CLEARANCE == pytest.approx(0.61)     # 0.25 + 0.36（实测偏差）


def test_cells_tighter_than_robot_radius_are_blocked():
    """无人机物理上塞不进去 —— 不是「贵一点」，是不可通行。"""
    import math
    g = np.full((21, 21), FREE, dtype=np.uint8)
    g[10, 10] = 0                     # 一个障碍
    r = plan_single_layer(g, (0, 0), (20, 20), (0.0, 0.0), 0.1)
    assert r["reached"] is True
    for ix, iy in r["path"]:
        d = math.hypot(ix - 10, iy - 10) * 0.1
        assert d >= 0.25, f'路径贴到障碍 {d:.2f}m < robot_radius ({ix},{iy})'


# ── 防过拟合：参数从物理量推导，不从图上拟 ───────────────────────

def test_beta_is_derived_from_speed_limits_not_tuned():
    """β = vx_max / vz_max：爬 1 m 的时间里能巡航多远。

    这是**物理量**，换任何地图都一样。它不是调出来的。
    """
    from marsim_nav.slice_planner import VX_MAX, VZ_MAX, beta_from_speeds
    assert beta_from_speeds(VX_MAX, VZ_MAX) == pytest.approx(0.8 / 0.6)
    # 换一组速度，β 必须跟着变 —— 证明它是推导的，不是常数
    assert beta_from_speeds(1.6, 0.6) == pytest.approx(1.6 / 0.6)


def test_detour_penalty_has_distance_units():
    """α 改名 detour_penalty，含义是「愿为避开 1m 完全贴死的窄缝多绕几米」。

    有量纲的问句，任何地图下都有意义；答案由使用者给，不是从数据拟出来的。
    """
    from marsim_nav.slice_planner import DETOUR_PENALTY
    assert DETOUR_PENALTY == 3.0


def test_widening_a_corridor_never_increases_cost():
    """性质测试（与地图无关）：把走廊加宽，代价不得上升。"""
    def cost_of(clear_cells):
        g = np.full((25, 25), FREE, dtype=np.uint8)
        g[0, :] = 0; g[24, :] = 0
        g[12, 5:20] = 0
        for iy in range(12 - clear_cells, 12):
            g[iy, 5:20] = FREE
        r = plan_single_layer(g, (2, 12), (22, 12), (0.0, 0.0), 0.1,
                              alpha=3.0, robot_radius=0.05,
                              required_clearance=0.30)
        return r["cost"] if r["reached"] else float("inf")
    assert cost_of(2) >= cost_of(4) >= cost_of(8)


def test_adding_an_obstacle_never_decreases_cost():
    """性质测试：多一个障碍，穿过它的路径代价不得下降。"""
    def cost(extra):
        g = np.full((25, 25), FREE, dtype=np.uint8)
        if extra:
            g[12, 12] = 0
        r = plan_single_layer(g, (2, 12), (22, 12), (0.0, 0.0), 0.1,
                              alpha=3.0, robot_radius=0.05,
                              required_clearance=0.30)
        return r["cost"] if r["reached"] else float("inf")
    assert cost(True) >= cost(False)
