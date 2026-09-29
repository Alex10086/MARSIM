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


def test_blocked_segment_must_be_rechecked_not_skipped():
    """受阻的段不得因「已检查过」而被跳过 —— 那会让拒绝变成一次性摆设。"""
    from marsim_nav.alt_profile import needs_corridor_check

    # 首次：要查
    assert needs_corridor_check(0, 1.0, 10.0, cleared=set(),
                                last_check=None, now=0.0) is True
    # 受阻后（未加入 cleared）：仍要查，但受 retry_interval 节流
    assert needs_corridor_check(0, 1.0, 10.0, cleared=set(),
                                last_check=100.0, now=101.0) is False   # 未到 2s
    assert needs_corridor_check(0, 1.0, 10.0, cleared=set(),
                                last_check=100.0, now=102.5) is True    # 到点重查


def test_cleared_segment_is_never_rechecked():
    from marsim_nav.alt_profile import needs_corridor_check

    assert needs_corridor_check(0, 1.0, 10.0, cleared={0},
                                last_check=100.0, now=999.0) is False


def test_no_check_when_already_at_target_height():
    from marsim_nav.alt_profile import needs_corridor_check

    assert needs_corridor_check(0, 10.0, 10.0, cleared=set(),
                                last_check=None, now=0.0) is False
