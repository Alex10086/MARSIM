import pytest

from marsim_nav.alt_profile import load_profile, next_vz


def test_climbs_upward_when_target_above():
    assert next_vz(1.0, 5.0) == pytest.approx(0.6)     # 0.8*4 = 3.2 -> 钳到 vz_max


def test_proportional_when_close():
    assert next_vz(1.0, 1.3) == pytest.approx(0.24)    # 0.8*0.3


def test_arrives_within_tolerance_stops():
    assert next_vz(1.0, 1.1) == 0.0


def test_descends_with_negative_vz():
    assert next_vz(9.0, 1.0) == pytest.approx(-0.6)


def test_clamps_vz_max():
    assert abs(next_vz(1.0, 50.0)) <= 0.6


def test_load_profile_returns_segment_dicts(tmp_path):
    p = tmp_path / 'p.yaml'
    p.write_text(
        'frame: world\n'
        'segments:\n'
        '  - {x: 5.0, y: 0.0, z: 10.0}\n'
        '  - {x: 20.0, y: -3.0, z: 10.0}\n', encoding='utf8')
    segs = load_profile(str(p))
    assert len(segs) == 2
    assert segs[0] == {'x': 5.0, 'y': 0.0, 'z': 10.0, 'note': ''}
    assert segs[1]['z'] == 10.0


def test_load_profile_rejects_missing_xyz(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('segments:\n  - {x: 1.0, y: 2.0}\n', encoding='utf8')
    with pytest.raises(ValueError):
        load_profile(str(p))
