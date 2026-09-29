"""切片图 A*：在 (x, y, 层号) 上搜出该飞哪一层、在哪升降。

纯函数，不 import rclpy。代价设计见 spec §2：

    层内 8 邻接  cost = 距离 × (1 + α × 该层膨胀后占用)
    层间换层    cost = β × |Δz|，前置条件是该处垂直走廊通畅

启发式取「欧氏距离 × (1 + α × 各层最小占用)」—— 它低估真实代价
（真实路径的占用不可能低于全局最小），所以是可采纳的，
A* 返回的仍是最优解。
"""
import heapq
import math

import numpy as np


def world_to_grid(x, y, origin, res):
    """世界坐标 -> 栅格坐标（向下取整，与 map_projector 的写法一致）。"""
    return (int(math.floor((x - origin[0]) / res)),
            int(math.floor((y - origin[1]) / res)))


def grid_to_world(ix, iy, origin, res):
    """栅格坐标 -> 该格中心的世界坐标。"""
    return (origin[0] + (ix + 0.5) * res, origin[1] + (iy + 0.5) * res)


def _neighbours(ix, iy, w, h):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = ix + dx, iy + dy
            if 0 <= nx < w and 0 <= ny < h:
                yield nx, ny, math.hypot(dx, dy)


def _empty(cost, length, reached):
    return {"path": [], "cost": float(cost), "length": float(length),
            "min_clearance": 0.0, "reached": bool(reached)}


def plan_single_layer(grid, start_ij, goal_ij, origin, res, alpha=2.0):
    """单层 A*。start/goal 是 (ix, iy)。

    **栅格语义与 map_projector 一致：0 = 障碍，254 = 自由。**
    （PGM 里 0 是黑色即障碍。这里曾写反过，导致自由与障碍互换。）

    返回 {"path": [(ix,iy),...], "cost", "length", "min_clearance", "reached"}。
    不可达时 reached=False、path=[]，不抛异常。
    """
    g = np.asarray(grid)
    if g.ndim != 2:
        raise ValueError(f'grid 必须是二维数组，实际 shape={g.shape}')
    h, w = g.shape
    sx, sy = int(start_ij[0]), int(start_ij[1])
    gx, gy = int(goal_ij[0]), int(goal_ij[1])
    blocked = g == 0          # 0 = 障碍（与 map_projector 一致）

    if not (0 <= sx < w and 0 <= sy < h) or not (0 <= gx < w and 0 <= gy < h):
        return _empty(0.0, 0.0, False)
    if blocked[sy, sx] or blocked[gy, gx]:
        return _empty(0.0, 0.0, False)

    # 占用率：254(自由) -> 0，越接近 0(障碍) 越高
    occ = 1.0 - g.astype(np.float64) / 255.0
    step_cost = 1.0 + float(alpha) * occ

    def hcost(x, y):
        return math.hypot(x - gx, y - gy)

    dist = {(sx, sy): 0.0}
    prev = {}
    pq = [(hcost(sx, sy), (sx, sy))]
    seen = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in seen:
            continue
        seen.add(cur)
        if cur == (gx, gy):
            break
        cx, cy = cur
        for nx, ny, d in _neighbours(cx, cy, w, h):
            if blocked[ny, nx]:
                continue
            nd = dist[cur] + d * float(step_cost[ny, nx])
            if nd < dist.get((nx, ny), math.inf):
                dist[(nx, ny)] = nd
                prev[(nx, ny)] = cur
                heapq.heappush(pq, (nd + hcost(nx, ny), (nx, ny)))

    if (gx, gy) not in dist:
        return _empty(0.0, 0.0, False)

    path = [(gx, gy)]
    while path[-1] != (sx, sy):
        path.append(prev[path[-1]])
    path.reverse()

    length = sum(math.hypot(path[i + 1][0] - path[i][0],
                            path[i + 1][1] - path[i][1])
                 for i in range(len(path) - 1)) * float(res)
    return {"path": path, "cost": float(dist[(gx, gy)]), "length": float(length),
            "min_clearance": 0.0, "reached": True}
