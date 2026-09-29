import numpy as np

from marsim_nav.gen_layers import DEFAULT_BANDS, generate_layers


def _wall(z_lo, z_hi, x=1.0):
    z = np.linspace(z_lo, z_hi, 12)
    return np.stack([np.full_like(z, x), np.zeros_like(z), z], axis=1)


def test_default_bands_are_seven_and_ascending():
    assert len(DEFAULT_BANDS) == 7
    assert all(a < b for a, b in DEFAULT_BANDS)
    assert all(DEFAULT_BANDS[i][1] <= DEFAULT_BANDS[i + 1][0]
               for i in range(len(DEFAULT_BANDS) - 1))


def test_each_band_gets_its_own_map(tmp_path):
    pts = np.vstack([_wall(0.5, 1.5), _wall(9.6, 11.5)])
    written = generate_layers(pts, DEFAULT_BANDS, res=0.1, margin=1.0,
                              inflation_radius=0.0,
                              out_dir=str(tmp_path), name='t')
    assert len(written) == len(DEFAULT_BANDS)
    assert all((tmp_path / p).exists() for p in written)


def test_only_bands_containing_points_are_occupied(tmp_path):
    pts = _wall(9.6, 11.5)          # 只落在 L6 (9.5~12.0)
    generate_layers(pts, DEFAULT_BANDS, res=0.1, margin=1.0,
                    inflation_radius=0.0, out_dir=str(tmp_path), name='t')
    for i, (lo, hi) in enumerate(DEFAULT_BANDS):
        pgm = (tmp_path / f't_L{i}.pgm').read_bytes()
        occupied = pgm.count(b'\x00')       # PGM: 0 = 障碍
        if 9.5 <= lo and hi <= 12.0:
            assert occupied > 0, f'L{i} 应有障碍'
        else:
            assert occupied == 0, f'L{i} 应为空'
