"""从一个点云生成多张高度切片的全局地图（PGM + YAML）。

复用 marsim_nav.map_projector，不改它。高度带用世界系（米）。
"""
import os

import numpy as np

from marsim_nav.map_projector import project, write_pgm, write_yaml

# 与 spec 第 1 节的测量表一一对应，固定值以保证实验可复现。
DEFAULT_BANDS = [
    (0.4, 1.6), (1.6, 2.6), (2.6, 3.6), (3.6, 5.0),
    (5.0, 7.0), (7.0, 9.5), (9.5, 12.0),
]


def generate_layers(points_xyz, bands, res, margin, inflation_radius,
                    out_dir, name):
    """为每个高度带写一对 PGM/YAML，返回 YAML 路径列表。

    所有层共用同一套栅格范围（`project()` 按**整个点云**定 extent），
    所以各层可以直接叠加对比。
    """
    pts = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for i, (lo, hi) in enumerate(bands):
        result = project(pts, res, lo, hi, margin, inflation_radius)
        img = f'{name}_L{i}.pgm'
        write_pgm(os.path.join(out_dir, img), result.grid)
        write_yaml(os.path.join(out_dir, f'{name}_L{i}.yaml'),
                   img, result.resolution, result.origin)
        written.append(f'{name}_L{i}.yaml')
    return written


def main(args=None):
    """用法：ros2 run marsim_nav gen_layers -- <cloud.pcd> <out_dir> [name]"""
    import sys
    from marsim_nav.pcd_io import read_pcd

    argv = sys.argv[1:] if args is None else list(args)
    if len(argv) < 2:
        print(main.__doc__)
        return 1
    pts = read_pcd(argv[0])
    out_dir = argv[1]
    name = argv[2] if len(argv) > 2 else 'layer'
    written = generate_layers(pts, DEFAULT_BANDS, res=0.1, margin=1.0,
                              inflation_radius=0.0, out_dir=out_dir, name=name)
    for w in written:
        print(w, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
