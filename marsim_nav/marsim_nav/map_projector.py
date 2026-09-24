"""Project a world-frame point cloud into a 2D occupancy grid (PGM + YAML).

The height band is the load-bearing parameter: the full cloud contains the
ground plane (which would wall the robot in) and canopy (which it flies under),
so only points at flight level may become obstacles.
"""
from dataclasses import dataclass

import numpy as np

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover
    ndimage = None

FREE = 254
OCCUPIED = 0


@dataclass
class OccupancyResult:
    grid: np.ndarray
    origin: tuple
    resolution: float


def project(points_xyz, res, z_min, z_max, margin, inflation_radius):
    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.size == 0:
        raise ValueError('empty point cloud: refusing to write a blank map')

    band = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]

    x0 = float(pts[:, 0].min() - margin)
    y0 = float(pts[:, 1].min() - margin)
    x1 = float(pts[:, 0].max() + margin)
    y1 = float(pts[:, 1].max() + margin)

    w = int(np.ceil((x1 - x0) / res))
    h = int(np.ceil((y1 - y0) / res))
    grid = np.full((h, w), FREE, dtype=np.uint8)

    if band.size > 0:
        col = np.clip(np.floor((band[:, 0] - x0) / res).astype(np.int64), 0, w - 1)
        row = np.clip(np.floor((band[:, 1] - y0) / res).astype(np.int64), 0, h - 1)
        grid[(h - 1) - row, col] = OCCUPIED   # image row 0 is +y max

    if inflation_radius > 0.0:
        if ndimage is None:
            raise RuntimeError('scipy is required when inflation_radius > 0')
        dist = ndimage.distance_transform_edt(grid != OCCUPIED) * res
        grid[dist < inflation_radius] = OCCUPIED

    return OccupancyResult(grid=grid, origin=(x0, y0), resolution=float(res))


def write_pgm(path, grid):
    h, w = grid.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(grid.astype(np.uint8).tobytes())


def write_yaml(path, image_name, resolution, origin):
    with open(path, 'w') as f:
        f.write(
            f'image: {image_name}\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.196\n'
        )
