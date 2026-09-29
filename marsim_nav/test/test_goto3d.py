import math

import pytest

from marsim_nav.goto3d import goal_reached_3d


def test_reached_in_3d_when_both_within_tolerance():
    assert goal_reached_3d((1.0, 2.0, 1.0), (1.1, 2.1, 1.2)) is True


def test_not_reached_when_only_xy_close_but_z_far():
    """水平到了但高度没到 —— 不算到达。这正是「发三维点」要的语义。"""
    assert goal_reached_3d((1.0, 2.0, 1.0), (1.0, 2.0, 8.0)) is False


def test_not_reached_when_only_z_close_but_xy_far():
    assert goal_reached_3d((1.0, 2.0, 8.0), (10.0, 20.0, 8.0)) is False


def test_boundary_is_inclusive():
    assert goal_reached_3d((0.0, 0.0, 1.0), (0.35, 0.0, 1.3)) is True
    assert goal_reached_3d((0.0, 0.0, 1.0), (0.36, 0.0, 1.3)) is False


def test_z_tolerance_is_independent_of_xy_tolerance():
    """两个容差独立：高度容差 0.3m 不受水平 0.35m 影响。"""
    assert goal_reached_3d((0.0, 0.0, 1.0), (0.0, 0.0, 1.3)) is True
    assert goal_reached_3d((0.0, 0.0, 1.0), (0.0, 0.0, 1.31)) is False
