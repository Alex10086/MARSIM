import numpy as np
import pytest

from marsim_nav.map_projector import project, write_pgm, write_yaml


def test_single_point_marks_exactly_one_cell():
    pts = np.array([[0.05, 0.05, 1.0]])
    r = project(pts, res=0.1, z_min=0.15, z_max=3.0, margin=1.0, inflation_radius=0.0)
    assert (r.grid == 0).sum() == 1     # 0 == occupied
    assert r.resolution == 0.1


def test_points_outside_z_band_are_dropped():
    pts = np.array([[0.05, 0.05, 0.02],   # ground
                    [0.05, 0.05, 9.0]])   # canopy above band
    r = project(pts, 0.1, 0.15, 3.0, 1.0, inflation_radius=0.0)
    assert (r.grid == 0).sum() == 0
    assert (r.grid == 254).all()


def test_inflation_grows_occupied_area():
    pts = np.array([[0.0, 0.0, 1.0]])
    r0 = project(pts, 0.1, 0.15, 3.0, 1.0, inflation_radius=0.0)
    r1 = project(pts, 0.1, 0.15, 3.0, 1.0, inflation_radius=0.3)
    assert (r1.grid == 0).sum() > (r0.grid == 0).sum()


def test_origin_is_bbox_min_minus_margin():
    pts = np.array([[1.0, 2.0, 1.0], [3.0, 5.0, 1.0]])
    r = project(pts, 0.1, 0.15, 3.0, margin=1.0, inflation_radius=0.0)
    assert r.origin == pytest.approx((0.0, 1.0), abs=1e-6)


def test_pgm_is_binary_and_yaml_has_required_keys(tmp_path):
    # Point at (1,1) with margin=1.0 gives origin (0.0, 0.0), so the hardcoded
    # origin needle below is actually achievable for this input.
    pts = np.array([[1.0, 1.0, 1.0]])
    r = project(pts, 0.5, 0.15, 3.0, 1.0, 0.0)
    pgm, yml = tmp_path / 'm.pgm', tmp_path / 'm.yaml'
    write_pgm(str(pgm), r.grid)
    write_yaml(str(yml), 'm.pgm', r.resolution, r.origin)
    assert pgm.read_bytes().startswith(b'P5')
    text = yml.read_text()
    for needle in ('image: m.pgm', 'resolution: 0.5', '[0.0, 0.0, 0.0]'):
        assert needle in text


def test_empty_cloud_raises_rather_than_writing_a_blank_map():
    # An empty map would make Nav2 plan confidently through everything.
    with pytest.raises(ValueError):
        project(np.zeros((0, 3)), 0.1, 0.15, 3.0, 1.0, 0.0)
