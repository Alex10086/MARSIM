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
    g[2:18, 10] = 0                        # 一堵墙（0 = 障碍），缺口在 iy=5
    g[5, 10] = FREE
    r = plan_single_layer(g, (1, 10), (18, 10), (-1.0, -1.0), 0.1)
    assert r["path"][-1] == (18, 10)
    assert any(iy == 5 for _, iy in r["path"])           # 必须走那个缺口
    assert all(g[iy, ix] == FREE for ix, iy in r["path"])   # 没穿墙


def test_unreachable_goal_reports_cleanly():
    g = np.full((10, 10), FREE, dtype=np.uint8)
    g[5, :] = 0                            # 完全隔断（0 = 障碍）
    r = plan_single_layer(g, (1, 1), (1, 8), (-1.0, -1.0), 0.1)
    assert r["reached"] is False
    assert r["path"] == []


def test_higher_cost_cells_are_avoided():
    """alpha 起作用：直线是高代价时，宁可绕远路。

    反证：把 alpha 设成 0（只看距离）就直走 —— 两条一起钉住因果。
    """
    g = np.full((5, 20), FREE, dtype=np.uint8)
    g[1, 2:18] = 60                        # 直行那条高占用（step_cost≈2.5）

    r = plan_single_layer(g, (1, 1), (18, 1), (-1.0, -1.0), 0.1, alpha=2.0)
    straight = [iy for _, iy in r["path"]].count(1)
    assert straight < 10                   # 大部分路绕开了高代价行

    r0 = plan_single_layer(g, (1, 1), (18, 1), (-1.0, -1.0), 0.1, alpha=0.0)
    straight0 = [iy for _, iy in r0["path"]].count(1)
    assert straight0 > straight            # alpha=0 时不绕，因果成立


def test_path_is_optimal_on_uniform_grid():
    """启发式必须可采纳：均匀空地上应给出几何最短路（曼哈顿折线的对角版本）。"""
    g = np.full((30, 30), FREE, dtype=np.uint8)
    r = plan_single_layer(g, (2, 2), (27, 27), (0.0, 0.0), 0.1, alpha=2.0)
    # 25x25 的对角直线，8 邻接下每步 sqrt(2)
    assert r["length"] == pytest.approx(25 * np.sqrt(2) * 0.1, rel=1e-6)
