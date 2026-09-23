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

# ── Horizontal speed governor ─────────────────────────────────────────
# The position PD controller has no explicit speed setpoint: speed emerges as
# (KP_XY/KD_XY)*distance and, with the 5 m carrot, settles around 2.5 m/s.
# `limit_horizontal_speed` enforces a hard ceiling by stripping the component of
# horizontal acceleration that would increase speed beyond max_horiz_speed.
from quad_pid.geometry import limit_horizontal_speed


def test_speed_governor_noop_below_limit():
    ax, ay = limit_horizontal_speed(1.0, 2.0, 0.5, 0.0, max_speed=2.0)
    assert (ax, ay) == (1.0, 2.0)


def test_speed_governor_removes_outward_accel_at_limit():
    # Moving +x at 2.0 m/s (= limit) and commanded to accelerate +x → stripped.
    ax, ay = limit_horizontal_speed(1.0, 0.0, 2.0, 0.0, max_speed=2.0)
    assert ax == pytest.approx(0.0, abs=1e-9)
    assert ay == pytest.approx(0.0, abs=1e-9)


def test_speed_governor_keeps_braking_accel():
    # Moving +x at 2.0 m/s, commanded to decelerate (-x) → allowed.
    ax, ay = limit_horizontal_speed(-1.0, 0.0, 2.0, 0.0, max_speed=2.0)
    assert ax == pytest.approx(-1.0, abs=1e-9)


def test_speed_governor_keeps_perpendicular_accel():
    # Moving +x at limit, accel purely +y (perpendicular, no speed increase).
    ax, ay = limit_horizontal_speed(0.0, 1.0, 2.0, 0.0, max_speed=2.0)
    assert ax == pytest.approx(0.0, abs=1e-9)
    assert ay == pytest.approx(1.0, abs=1e-9)


def test_speed_governor_partial_strip_along_diagonal():
    # Moving along (1,1)/√2 slightly above the limit; accel (1,0) → only the
    # outward part is removed. (Speed chosen clearly above the limit to avoid
    # floating-point boundary ambiguity.)
    v = 1.5                                   # per-axis → speed 2.12 > 2.0
    ax, ay = limit_horizontal_speed(1.0, 0.0, v, v, max_speed=2.0)
    assert ax == pytest.approx(0.5, abs=1e-9)
    assert ay == pytest.approx(-0.5, abs=1e-9)


def test_speed_governor_noop_at_rest():
    ax, ay = limit_horizontal_speed(3.0, 3.0, 0.0, 0.0, max_speed=2.0)
    assert (ax, ay) == (3.0, 3.0)
