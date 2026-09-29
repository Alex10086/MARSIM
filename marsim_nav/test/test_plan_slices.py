import numpy as np
import pytest

from marsim_nav.slice_planner import plan_slices, world_to_grid

FREE = 254      # 0 = 障碍，254 = 自由（与 map_projector 一致）


def _free(h, w):
    return np.full((h, w), FREE, dtype=np.uint8)


def _wall_between(h, w, col):
    g = _free(h, w)
    g[:, col] = 0
    return g


def _empty_cloud():
    return np.zeros((0, 3), dtype=float)


def test_picks_the_free_layer():
    """下面那层有墙挡路，上面那层畅通 -> 应选上面那层。"""
    grids = [_wall_between(15, 15, 7), _free(15, 15)]
    zs = [1.0, 10.0]
    r = plan_slices(grids, zs, _empty_cloud(),
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1)
    assert r["reached"] is True
    # 目标的 z=1.0 -> 终点必须回到下层；但中途必须借道畅通的上层
    assert r["waypoints"][-1][2] == pytest.approx(1.0)
    assert any(abs(z - 10.0) < 1e-6 for _, _, z in r["waypoints"])
    # 并且换层必须发生在墙的两侧之外（不是穿墙）
    for i in range(len(r["waypoints"]) - 1):
        (x1, y1, z1), (x2, y2, z2) = r["waypoints"][i], r["waypoints"][i + 1]
        if abs(z1 - z2) < 1e-6:                        # 同层移动
            k = 0 if abs(z1 - 1.0) < 1e-6 else 1       # 按 z 判定是哪一层
            ix1, iy1 = world_to_grid(x1, y1, (0.0, 0.0), 0.1)
            assert grids[k][iy1, ix1] == FREE, f'第 {k} 层穿墙 ({x1},{y1})' 


def test_layer_change_costs_discourage_gratuitous_switching():
    """两层都通时，不该没事就换层（换层要爬升、要重规划）。"""
    grids = [_free(15, 15), _free(15, 15)]
    zs = [1.0, 10.0]
    r = plan_slices(grids, zs, _empty_cloud(),
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1, beta=1.5)
    layers = {round(z, 3) for _, _, z in r["waypoints"]}
    assert len(layers) == 1                    # 全程待在一层


def test_blocked_corridor_forbids_layer_change():
    """垂直走廊不通处不得换层 —— 否则会直接穿枝叶层。"""
    grids = [_wall_between(15, 15, 7), _free(15, 15)]
    zs = [1.0, 10.0]
    # 在唯一的换层点 (0.75, 0.75) 上方 z=5 放一个障碍 -> 走廊不通
    cloud = np.array([[0.75, 0.75, 5.0]])
    r = plan_slices(grids, zs, cloud,
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1, radius=0.25)
    # 走廊被封 -> 只要还能在别处换层就仍可达；这里只断言绝不在受阻点换层
    for i in range(len(r["waypoints"]) - 1):
        (x1, y1, z1), (x2, y2, z2) = r["waypoints"][i], r["waypoints"][i + 1]
        if abs(z1 - z2) > 1e-6:                # 这一步是换层
            assert not (abs(x1 - 0.75) < 0.05 and abs(y1 - 0.75) < 0.05), \
                f'在受阻点 ({x1},{y1}) 换层了'


def test_unreachable_reports_cleanly():
    grids = [_wall_between(15, 15, 7), _wall_between(15, 15, 7)]
    zs = [1.0, 10.0]
    r = plan_slices(grids, zs, _empty_cloud(),
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1)
    assert r["reached"] is False
    assert r["waypoints"] == []


def test_layer_costs_are_reported():
    """可解释性是必需的：要能回答「为什么选这层」。"""
    grids = [_wall_between(15, 15, 7), _free(15, 15)]
    zs = [1.0, 10.0]
    r = plan_slices(grids, zs, _empty_cloud(),
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1)
    assert len(r["layer_costs"]) == 2
    for lc in r["layer_costs"]:
        assert {"layer", "z", "reachable"} <= set(lc)


def test_waypoints_are_world_coords_with_layer_altitude():
    grids = [_free(15, 15), _free(15, 15)]
    zs = [1.0, 10.0]
    r = plan_slices(grids, zs, _empty_cloud(),
                    start_xyz=(0.15, 0.75, 1.0), goal_xyz=(1.35, 0.75, 1.0),
                    origin=(0.0, 0.0), res=0.1)
    assert r["waypoints"][0] == pytest.approx((0.15, 0.75, 1.0))
    for x, y, z in r["waypoints"]:
        assert 0.0 <= x <= 1.5 and 0.0 <= y <= 1.5
        assert z in (1.0, 10.0)                # z 必须是某一层的飞行高度
