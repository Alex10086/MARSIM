import numpy as np
import pytest

from marsim_nav.pcd_io import read_pcd


def _write_pcd(path, mode, xyz):
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    n = len(xyz)
    if mode == 'ascii':
        body = ''.join(f'{x} {y} {z}\n' for x, y, z in xyz)
        head = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                f'TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n'
                f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n')
    else:
        body = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
        head = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                f'TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n'
                f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n')
    raw = body.encode('ascii') if isinstance(body, str) else body
    path.write_bytes(head.encode('ascii') + raw)


def test_read_binary_pcd(tmp_path):
    f = tmp_path / 'a.pcd'
    _write_pcd(f, 'binary', [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    out = read_pcd(str(f))
    assert out.shape == (2, 3)
    assert out[0] == pytest.approx([1.0, 2.0, 3.0])
    assert out[1] == pytest.approx([4.0, 5.0, 6.0])


def test_read_ascii_pcd(tmp_path):
    f = tmp_path / 'b.pcd'
    _write_pcd(f, 'ascii', [(1.5, -2.5, 0.25)])
    assert read_pcd(str(f)) == pytest.approx(np.array([[1.5, -2.5, 0.25]]))


def test_read_pcd_rejects_non_pcd(tmp_path):
    f = tmp_path / 'c.pcd'
    f.write_bytes(b'not a pcd at all')
    with pytest.raises(ValueError):
        read_pcd(str(f))


def test_read_pcd_uses_field_names_not_column_order(tmp_path):
    """真实 PCD 是 `FIELDS rgb x y z _` —— 取前 3 列会拿到 rgb。"""
    f = tmp_path / 'd.pcd'
    n = 1
    head = ('# .PCD v0.7\nVERSION 0.7\nFIELDS rgb x y z _\n'
            'SIZE 4 4 4 4 1\nTYPE F F F F U\nCOUNT 1 1 1 1 4\n'
            f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {n}\nDATA ascii\n')
    f.write_bytes((head + '42.0 7.5 -8.25 0.5 0 0 0\n').encode('ascii'))
    out = read_pcd(str(f))
    assert out == pytest.approx(np.array([[7.5, -8.25, 0.5]]))
