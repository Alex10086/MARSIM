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


def test_blocked_state_forces_vz_zero_no_matter_the_height_error():
    """安全不变量：只要受阻，vz 必须为 0。

    这条曾经失守：检查只在「做检查的那个 tick」发 vz=0，其余 tick 照常
    按高度误差发爬升指令，结果日志写「保持高度」的同时 z 一路涨到 9.9m。
    """
    from marsim_nav.alt_profile import commanded_vz

    # 巨大的爬升需求，但受阻 -> 必须 0
    assert commanded_vz(True, 1.0, 10.0) == 0.0
    assert commanded_vz(True, 9.0, 1.0) == 0.0     # 降落同样被挡
    # 未受阻 -> 正常按高度误差输出
    assert commanded_vz(False, 1.0, 10.0) == pytest.approx(0.6)


def test_blocked_state_persists_until_a_clear_recheck():
    """受阻状态必须持续到重查通过为止，而不是只挡一个 tick。"""
    from marsim_nav.alt_profile import needs_corridor_check

    cleared, blocked = set(), True
    # 受阻后 3 秒内不重查：状态保持受阻
    for now in (0.0, 0.5, 1.0, 1.5):
        assert needs_corridor_check(0, 1.0, 10.0, cleared, 0.0, now) is False
        assert blocked is True
    # 到点重查 -> 通过 -> 进 cleared，受阻解除
    assert needs_corridor_check(0, 1.0, 10.0, cleared, 0.0, 2.5) is True
    cleared.add(0); blocked = False
    assert needs_corridor_check(0, 1.0, 10.0, cleared, 2.5, 99.0) is False
    assert blocked is False
