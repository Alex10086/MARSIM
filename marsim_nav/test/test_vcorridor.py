import math
import numpy as np
import pytest

from marsim_nav.vcorridor import check_vertical_corridor


def _pts(*xyz):
    return np.asarray(xyz, dtype=float).reshape(-1, 3)


def test_empty_cloud_is_clear():
    r = check_vertical_corridor(_pts(), 0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["clear"] is True
    assert r["blocking_z"] == []
    assert r["nearest"] == math.inf


def test_obstacle_outside_radius_does_not_block():
    r = check_vertical_corridor(_pts((0.30, 0.0, 5.0)), 0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["clear"] is True
    assert r["nearest"] == pytest.approx(0.30)


def test_obstacle_exactly_on_radius_blocks():
    r = check_vertical_corridor(_pts((0.25, 0.0, 5.0)), 0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["clear"] is False
    assert r["blocking_z"] == [(5.0, 5.0)]


def test_obstacle_outside_vertical_range_is_ignored():
    r = check_vertical_corridor(_pts((0.0, 0.0, 0.5), (0.0, 0.0, 9.5)),
                                0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["clear"] is True


def test_reversed_z_range_is_normalised():
    r = check_vertical_corridor(_pts((0.0, 0.0, 5.0)), 0.0, 0.0, 9.0, 1.0, 0.25)
    assert r["clear"] is False


def test_adjacent_blocking_points_merge_into_one_interval():
    r = check_vertical_corridor(_pts((0.0, 0.0, 5.0), (0.0, 0.0, 5.2)),
                                0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["blocking_z"] == [(5.0, 5.2)]


def test_separate_blocking_points_yield_separate_intervals():
    r = check_vertical_corridor(_pts((0.0, 0.0, 5.0), (0.0, 0.0, 7.5)),
                                0.0, 0.0, 1.0, 9.0, 0.25)
    assert r["blocking_z"] == [(5.0, 5.0), (7.5, 7.5)]
