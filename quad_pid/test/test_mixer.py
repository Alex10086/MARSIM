import numpy as np
import pytest
from quad_pid.mixer import allocate

K_F = 2.6944e-8
K_T = 1.049e-10
ARM = 0.22
MASS = 1.9
G = 9.81


def test_hover_all_equal():
    """Hover thrust should produce equal RPM across all 4 motors."""
    F_hover = MASS * G  # 18.639 N
    rpm = allocate(F_hover, 0.0, 0.0, 0.0, K_F, K_T, ARM)
    assert len(rpm) == 4
    assert rpm[0] == pytest.approx(rpm[1], rel=0.01)
    assert rpm[1] == pytest.approx(rpm[2], rel=0.01)
    assert rpm[2] == pytest.approx(rpm[3], rel=0.01)
    # Hover RPM should be ~13150
    assert rpm[0] == pytest.approx(13150, rel=0.05)


def test_roll_torque_antisymmetric():
    """Pure roll torque: left motors increase, right motors decrease."""
    F_hover = MASS * G
    rpm = allocate(F_hover, 0.5, 0.0, 0.0, K_F, K_T, ARM)
    # Roll torque: motor 0 and 3 (right side) decrease,
    # motor 1 and 2 (left side) increase
    assert rpm[0] < rpm[2]  # right vs left
    assert rpm[3] < rpm[1]


def test_yaw_torque_splits_cw_ccw():
    """Pure yaw torque: CW motors change opposite to CCW motors."""
    F_hover = MASS * G
    rpm = allocate(F_hover, 0.0, 0.0, 0.1, K_F, K_T, ARM)
    # Motors 0,1 (CW) decrease; motors 2,3 (CCW) increase
    avg_cw = (rpm[0] + rpm[1]) / 2
    avg_ccw = (rpm[2] + rpm[3]) / 2
    assert avg_ccw > avg_cw  # positive yaw torque → CCW speed up


def test_zero_thrust_all_zero():
    """Zero thrust + zero torque → all RPMs zero."""
    rpm = allocate(0.0, 0.0, 0.0, 0.0, K_F, K_T, ARM)
    assert np.all(rpm == 0.0)