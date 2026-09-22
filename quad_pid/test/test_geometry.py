import math
import pytest
from quad_pid.geometry import accel_to_attitude, yaw_from_quaternion, shortest_angle

G = 9.81


def test_hover_accel_zero_tilt():
    """Pure hover: a_des = [0,0,g] → zero roll and pitch."""
    roll, pitch = accel_to_attitude(0.0, 0.0, G)
    assert roll == pytest.approx(0.0, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-6)


def test_forward_accel_positive_pitch():
    """Forward acceleration → positive pitch (nose down)."""
    roll, pitch = accel_to_attitude(2.0, 0.0, G)
    assert pitch > 0.0       # nose down = positive pitch
    assert pitch < 0.35      # within 20° for modest accel
    assert roll == pytest.approx(0.0, abs=1e-6)


def test_right_accel_positive_roll():
    """Right acceleration → negative roll (right side down)
    given body x=forward, y=left."""
    roll, pitch = accel_to_attitude(0.0, 2.0, G)
    assert roll < 0.0         # right accel → tilt right, roll negative
    assert pitch == pytest.approx(0.0, abs=1e-6)


def test_yaw_from_identity_quaternion():
    """Identity quaternion → yaw = 0."""
    yaw = yaw_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert yaw == pytest.approx(0.0, abs=1e-6)


def test_shortest_angle_across_pi():
    """Crossing ±π should wrap to short path."""
    d = shortest_angle(math.pi - 0.1, -math.pi + 0.1)
    assert d == pytest.approx(-0.2, abs=0.01)


def test_shortest_angle_no_wrap():
    """Small difference should not wrap."""
    d = shortest_angle(1.0, 0.5)
    assert d == pytest.approx(0.5, abs=0.01)