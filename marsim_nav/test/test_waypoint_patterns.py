import math

import pytest

from marsim_nav.waypoint_patterns import cube, figure8, serpentine


def test_cube_has_eight_distinct_corners_at_three_heights():
    pts = cube((0.0, 0.0, 1.0), 2.0)
    assert len(pts) == 8
    assert len(set(pts)) == 8
    zs = {round(z, 6) for _, _, z in pts}
    assert zs == {-1.0, 3.0}                 # cz ± half
    for x, y, z in pts:
        assert abs(abs(x) - 2.0) < 1e-9      # 都在角上
        assert abs(abs(y) - 2.0) < 1e-9
        assert abs(abs(z - 1.0) - 2.0) < 1e-9


def test_cube_is_deterministic():
    assert cube((1.0, 2.0, 3.0), 1.0) == cube((1.0, 2.0, 3.0), 1.0)


def test_serpentine_ramps_z_monotonically():
    pts = serpentine(0.0, 10.0, 0.0, 8.0, 1.0, 5.0, rows=3, cols=3)
    zs = [z for _, _, z in pts]
    assert zs == sorted(zs)
    assert zs[0] == pytest.approx(1.0) and zs[-1] == pytest.approx(5.0)


def test_serpentine_alternates_x_direction():
    pts = serpentine(0.0, 10.0, 0.0, 8.0, 1.0, 5.0, rows=3, cols=3)
    xs = [x for x, _, _ in pts]
    # 第一行 0 -> 10 -> 0（来回）
    assert xs[0] == pytest.approx(0.0)
    assert xs[1] == pytest.approx(10.0)
    assert xs[2] == pytest.approx(0.0)
    assert len({round(y, 6) for _, y, _ in pts}) == 3     # 3 行不同的 y


def test_serpentine_rejects_too_small_grid():
    with pytest.raises(ValueError):
        serpentine(0, 1, 0, 1, 0, 1, rows=1, cols=3)


def test_figure8_self_intersects_at_center_with_different_heights():
    """8 字自交于中心 —— 同一 (x,y) 但高度不同，这正是「三维点」的意义。"""
    pts = figure8((0.0, 0.0), 2.0, 1.0, 3.0, n=8)
    assert len(pts) == 8
    # t=0 与 t=pi 都在 (±2, 0)；t=pi/2 与 3pi/2 都在 (0,0) -> 自交
    zeros = [p for p in pts if abs(p[0]) < 1e-9 and abs(p[1]) < 1e-9]
    assert len(zeros) == 2
    assert abs(zeros[0][2] - zeros[1][2]) > 0.5           # 同一 XY、高度不同


def test_figure8_z_is_monotone_from_z0_to_z1():
    pts = figure8((0.0, 0.0), 1.0, 1.0, 5.0, n=12)
    zs = [z for _, _, z in pts]
    assert zs == sorted(zs)
    assert zs[0] == pytest.approx(1.0) and zs[-1] == pytest.approx(5.0)


def test_figure8_rejects_too_few_points():
    with pytest.raises(ValueError):
        figure8((0, 0), 1.0, 0.0, 1.0, n=3)


def test_all_patterns_stay_within_radius():
    """图案必须落在中心半径内 —— 别跑到地图外面去。"""
    for pts, cx, cy, r in [
        (cube((0, 0, 1), 2.0), 0, 0, 3.0),
        (serpentine(-2, 2, -2, 2, 1, 2, rows=3, cols=3), 0, 0, 3.5),
        (figure8((0, 0), 2.0, 1, 2, n=8), 0, 0, 2.5),
    ]:
        for x, y, _ in pts:
            assert math.hypot(x - cx, y - cy) <= r + 1e-9
