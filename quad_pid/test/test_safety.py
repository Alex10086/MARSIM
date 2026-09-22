import math
import numpy as np
import pytest

from quad_pid.mixer import allocate
from quad_pid.geometry import shortest_angle, accel_to_attitude

K_F = 2.6944e-8
K_T = 1.049e-10
ARM = 0.22
MASS = 1.9
G = 9.81


def test_mixer_no_nan_from_negative_thrust():
    """Review Focus #2: negative thrust should produce zeros, not NaN."""
    rpm = allocate(-100.0, 0.0, 0.0, 0.0, K_F, K_T, ARM)
    assert not np.any(np.isnan(rpm))
    assert np.all(rpm == 0.0)


def test_allocate_with_extreme_torque():
    """Extreme torque should not produce NaN."""
    F_hover = MASS * G
    rpm = allocate(F_hover, 1000.0, 1000.0, 1000.0, K_F, K_T, ARM)
    assert not np.any(np.isnan(rpm))


def test_shortest_angle_yaw_wrap_179_to_neg179():
    """Review Focus #4: yaw from 179° to -179° = -2° (not +358°)."""
    d = shortest_angle(math.radians(179), math.radians(-179))
    assert d == pytest.approx(math.radians(-2), abs=0.01)


def test_shortest_angle_yaw_wrap_neg179_to_179():
    """Reverse: yaw from -179° to 179° = +2°."""
    d = shortest_angle(math.radians(-179), math.radians(179))
    assert d == pytest.approx(math.radians(2), abs=0.01)


def test_shortest_angle_zero_diff():
    """Same angle → zero diff."""
    for a in [0.0, 1.0, -1.0, math.pi, -math.pi, 3.0]:
        assert shortest_angle(a, a) == pytest.approx(0.0, abs=1e-10)


def test_accel_to_attitude_degenerate_zero():
    """Review Focus #2: near-zero acceleration vector should not crash."""
    roll, pitch = accel_to_attitude(0.0, 0.0, 1e-10)
    assert not math.isnan(roll)
    assert not math.isnan(pitch)


def test_mixer_hover_zero_torque():
    """Review Focus #3: hover thrust with zero error → equal RPM."""
    F_hover = MASS * G
    rpm = allocate(F_hover, 0.0, 0.0, 0.0, K_F, K_T, ARM)
    rpm_expected = math.sqrt(F_hover / (4.0 * K_F))
    for r in rpm:
        assert r == pytest.approx(rpm_expected, rel=0.01)