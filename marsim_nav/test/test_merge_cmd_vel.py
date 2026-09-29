import math

import pytest

from marsim_nav.merge_cmd_vel import AltCmdState, merge_twist


def test_merges_horizontal_from_nav_and_vertical_from_alt():
    assert merge_twist(0.5, -0.2, 0.1, 0.3, alt_age=0.0) == (0.5, -0.2, 0.3, 0.1)


def test_stale_alt_forces_vz_zero():
    assert merge_twist(0.5, -0.2, 0.1, 0.3, alt_age=0.6)[2] == 0.0


def test_fresh_alt_at_timeout_edge_is_still_stale():
    assert merge_twist(0.0, 0.0, 0.0, 0.3, alt_age=0.5, timeout=0.5)[2] == 0.0


def test_non_finite_alt_is_zeroed():
    for bad in (math.nan, math.inf, -math.inf):
        assert merge_twist(0.5, 0.0, 0.0, bad, alt_age=0.0)[2] == 0.0


def test_non_finite_nav_component_is_zeroed():
    vx, vy, vz, wz = merge_twist(math.nan, 0.2, 0.3, 0.4, alt_age=0.0)
    assert (vx, vy, vz, wz) == (0.0, 0.2, 0.4, 0.3)


def test_negative_age_is_treated_as_fresh():
    assert merge_twist(0.0, 0.0, 0.0, 0.3, alt_age=-1.0)[2] == 0.3


def test_alt_state_tracks_last_stamp_and_age():
    st = AltCmdState()
    assert st.age(10.0) == math.inf        # 从未收到 -> 视为超时
    st.update(0.3, stamp=10.0)
    assert st.age(10.2) == pytest.approx(0.2)
    assert st.vz == pytest.approx(0.3)


def test_alt_state_age_is_elapsed_time_not_a_flag():
    st = AltCmdState()
    st.update(0.3, stamp=10.0)
    assert st.age(11.0) == pytest.approx(1.0)
