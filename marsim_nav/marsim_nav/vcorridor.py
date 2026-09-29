"""垂直走廊碰撞检查：给定水平位置与竖直区间，判断这条柱子是否通畅。

纯函数，不 import rclpy —— 这是全系统爬升/降落的唯一安全防线
（局部代价图跟随机体高度，看不到上方障碍直到你爬到那儿）。
"""
import math

import numpy as np


def check_vertical_corridor(points_xyz, x, y, z0, z1, radius):
    """判断 (x, y) 处、从 z0 到 z1 的竖直柱子是否通畅。

    水平距离 <= radius 的点算阻挡（含等于）；只考虑 z 落在
    [min(z0,z1), max(z0,z1)] 内的点（含端点）。

    报告的 blocking_z 是**区间**：相邻阻挡点的竖直间隙 <= radius 时归为同一段
    （对无人机而言，比自身半径还近的两个阻挡物就是同一处障碍）。间隙大于
    radius 才算两段——那中间是能钻过去的空档。
    """
    pts = np.asarray(points_xyz, dtype=float)
    lo, hi = (z0, z1) if z0 <= z1 else (z1, z0)

    empty = {"clear": True, "blocking_z": [], "nearest": math.inf}
    if pts.size == 0:
        return empty
    pts = pts.reshape(-1, 3)

    band = pts[(pts[:, 2] >= lo) & (pts[:, 2] <= hi)]
    if band.size == 0:
        return empty

    dist = np.hypot(band[:, 0] - x, band[:, 1] - y)
    hit = dist <= radius
    if not hit.any():
        return {"clear": True, "blocking_z": [], "nearest": float(dist.min())}

    zs = np.sort(band[hit, 2])
    intervals = [(float(zs[0]), float(zs[0]))]
    for z in zs[1:]:
        if z - intervals[-1][1] <= radius:
            intervals[-1] = (intervals[-1][0], float(z))
        else:
            intervals.append((float(z), float(z)))
    return {"clear": False, "blocking_z": intervals,
            "nearest": float(dist[hit].min())}


def main(args=None):
    """起飞前预检：对若干 (x, y, z0, z1, radius) 逐个查垂直走廊。

    用法：
        ros2 run marsim_nav vcorridor -- <cloud.pcd> <x> <y> <z0> <z1> [radius] [...]
    每 5 个一组；组数不限。全部通畅返回 0，任一受阻返回 1。
    """
    import sys
    from marsim_nav.pcd_io import read_pcd

    argv = sys.argv[1:] if args is None else list(args)
    if len(argv) < 6 or (len(argv) - 1) % 5 != 0:
        print(main.__doc__)
        return 1
    pts = read_pcd(argv[0])
    bad = 0
    for i in range(1, len(argv), 5):
        x, y, z0, z1, radius = (float(v) for v in argv[i:i + 5])
        r = check_vertical_corridor(pts, x, y, z0, z1, radius)
        flag = 'OK  ' if r['clear'] else 'BLOCK'
        print(f'{flag} ({x:.2f},{y:.2f}) {z0:.2f}->{z1:.2f} r={radius:.2f} '
              f'nearest={r["nearest"]:.2f} blocking={r["blocking_z"]}', flush=True)
        bad += 0 if r['clear'] else 1
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
