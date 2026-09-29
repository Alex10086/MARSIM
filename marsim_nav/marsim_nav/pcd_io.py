"""读 PCD v0.7（binary / ascii），返回 (N, 3) 的 float 数组。

按**字段名**取 x/y/z，不按列顺序 —— 真实地图 PCD 的 FIELDS 是
`rgb x y z _`，取前 3 列会拿到 rgb。
"""
import numpy as np

_BASE = {'F': {4: 'f4', 8: 'f8'},
         'U': {1: 'u1', 2: 'u2', 4: 'u4'},
         'I': {1: 'i1', 2: 'i2', 4: 'i4'}}


def _header(raw):
    he = raw.find(b'DATA ')
    if he < 0:
        raise ValueError('不是合法的 PCD（缺少 DATA 行）')
    hdr = raw[:he].decode('ascii', 'replace')
    def g(k):
        return next(l for l in hdr.splitlines() if l.startswith(k)).split()[1:]
    return he, g


def read_pcd(path):
    raw = open(path, 'rb').read()
    he, g = _header(raw)
    fields, sizes, types, counts = g('FIELDS'), g('SIZE'), g('TYPE'), g('COUNT')
    n = int(g('POINTS')[0])
    mode = raw[he:].split(b'\n', 1)[0].split()[1].decode('ascii')

    for want in ('x', 'y', 'z'):
        if want not in fields:
            raise ValueError(f'{path}: 缺少字段 {want!r}（FIELDS={fields}）')
    cols = [fields.index(w) for w in ('x', 'y', 'z')]

    if mode == 'ascii':
        body = raw[he:].split(b'\n', 1)[1].decode('ascii', 'replace')
        rows = [ln.split() for ln in body.splitlines() if ln.strip()]
        if not rows:
            return np.zeros((0, 3), dtype=float)
        arr = np.asarray(rows, dtype=float)
        return arr[:n, cols]

    if mode != 'binary':
        raise ValueError(f'{path}: 不支持 DATA {mode}')

    dt = []
    for f, sz, t, c in zip(fields, sizes, types, counts):
        sz, c = int(sz), int(c)
        dt.append((f, np.dtype(f'V{sz * c}') if c > 1 else np.dtype(_BASE[t][sz])))

    body = raw[he:].split(b'\n', 1)[1]
    pts = np.frombuffer(body, dtype=np.dtype(dt), count=n)
    return np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(float)
