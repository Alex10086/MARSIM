"""Tests for the world -> LiDAR re-expression used by cloud_reframe_node."""
import numpy as np
import pytest

from marsim_nav.cloud_transform import world_to_lidar

ZERO = np.zeros(3)
NO_OFFSET = np.zeros(3)


def _yaw(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_identity_leaves_points_alone():
    pts = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]])
    out = world_to_lidar(pts, np.eye(3), ZERO, NO_OFFSET)
    np.testing.assert_allclose(out, pts, atol=1e-12)


def test_vehicle_at_origin_sees_itself_at_zero():
    out = world_to_lidar(np.array([[0.0, 0.0, 0.0]]), np.eye(3), ZERO, NO_OFFSET)
    np.testing.assert_allclose(out, [[0.0, 0.0, 0.0]], atol=1e-12)


def test_translation_is_relative_to_the_vehicle():
    """A point sitting on the vehicle maps to the lidar's local origin."""
    out = world_to_lidar(np.array([[1.0, 2.0, 3.0]]), np.eye(3),
                         np.array([1.0, 2.0, 3.0]), NO_OFFSET)
    np.testing.assert_allclose(out, [[0.0, 0.0, 0.0]], atol=1e-12)


def test_lidar_offset_is_subtracted_in_body_frame():
    out = world_to_lidar(np.array([[0.0, 0.0, 0.0]]), np.eye(3), ZERO,
                         np.array([0.0, 0.0, 0.1]))
    np.testing.assert_allclose(out, [[0.0, 0.0, -0.1]], atol=1e-12)


def test_yaw_rotation_inverts_correctly():
    """A point 1 m ahead in world, with the vehicle yawed +90 deg, is to the
    vehicle's RIGHT in the body frame (i.e. body y = -1)."""
    out = world_to_lidar(np.array([[1.0, 0.0, 0.0]]), _yaw(np.pi / 2), ZERO,
                         NO_OFFSET)
    np.testing.assert_allclose(out, [[0.0, -1.0, 0.0]], atol=1e-12)


def test_round_trip_reconstructs_world_points():
    """p_w == R @ p_l + (t + R @ o), i.e. the frame composition Nav2 undoes."""
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(50, 3)) * 10.0
    rot = _yaw(0.7)
    trans = np.array([12.0, -3.0, 1.0])
    off = np.array([0.0, 0.0, 0.1])
    lidar = world_to_lidar(pts, rot, trans, off)
    back = (rot @ (lidar + off).T).T + trans
    np.testing.assert_allclose(back, pts, atol=1e-9)


def test_rejects_wrong_shape():
    with pytest.raises(ValueError):
        world_to_lidar(np.zeros((4, 2)), np.eye(3), ZERO, NO_OFFSET)
