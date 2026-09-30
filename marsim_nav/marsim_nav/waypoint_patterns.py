"""三维路点图案：用于 goto3d 的回归验证。

纯函数，不 import rclpy。三种图案各自测不同的运动特性：

- `cube`       立方体 8 角点 —— 大幅高度变化 + 方向急转
- `serpentine` 蛇形往返     —— 反复换向 + 连续升降
- `figure8`    双纽线 8 字  —— 连续曲率 + 路径自交
"""
import math


def cube(center, half):
    """以 center 为中心、边长 2*half 的立方体 8 个角点。

    返回顺序是确定的（z 外层、y、x 内层），便于复现。
    """
    cx, cy, cz = (float(v) for v in center)
    out = []
    for sz in (-1, 1):
        for sy in (-1, 1):
            for sx in (-1, 1):
                out.append((cx + sx * half, cy + sy * half, cz + sz * half))
    return out


def serpentine(x0, x1, y0, y1, z0, z1, rows=3, cols=2):
    """蛇形往返：x 来回扫，y 逐行推进，z 从 z0 线性升到 z1。"""
    if rows < 2 or cols < 2:
        raise ValueError('rows/cols 至少为 2')
    out = []
    n = rows * cols
    for i in range(n):
        r, c = divmod(i, cols)
        x = x0 if c % 2 == 0 else x1
        if r % 2 == 1:                     # 奇数行反向，形成 boustrophedon
            x = x1 if c % 2 == 0 else x0
        y = y0 + (y1 - y0) * r / (rows - 1)
        z = z0 + (z1 - z0) * i / (n - 1)
        out.append((x, y, z))
    return out


def figure8(center, radius, z0, z1, n=8):
    """Gerono 双纽线：x = cx + r*cos t，y = cy + r*sin t*cos t，z 线性升降。

    路径自交于中心，测连续曲率与「经过同一点但高度不同」。
    """
    if n < 4:
        raise ValueError('n 至少为 4')
    cx, cy = float(center[0]), float(center[1])
    out = []
    for i in range(n):
        t = 2 * math.pi * i / n
        out.append((cx + radius * math.cos(t),
                    cy + radius * math.sin(t) * math.cos(t),
                    z0 + (z1 - z0) * i / (n - 1)))
    return out
