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

try:
    from scipy import ndimage as _ndimage
except ImportError:      # pragma: no cover - scipy 已是 map_projector 的依赖
    _ndimage = None

# 安全量，由实测得出，不是调参：
#   robot_radius 0.25 m（机体物理半径 0.22 + 裕度）
#   期望跟踪偏差 0.36 m（KP_XY 4.0 修正后，长距离飞行实测的最大横向偏差）
ROBOT_RADIUS = 0.25
REQUIRED_CLEARANCE = 0.61

# ── 防过拟合：下面两个量都从物理量推，不从任何地图上拟 ────────────
# 巡航/爬升的速度上限（与 quad_pid_nav.yaml 的 max_horiz_speed / twist_max_vz 一致）
VX_MAX = 0.8
VZ_MAX = 0.6

# α 改名为 detour_penalty，含义是「愿为避开 1m 完全贴死的窄缝，多绕几米」。
# 有量纲的问句，任何地图下都有意义；答案由使用者给，不是从数据拟出来的。
DETOUR_PENALTY = 3.0


def beta_from_speeds(vx_max=VX_MAX, vz_max=VZ_MAX):
    """换层代价 β = vx_max / vz_max，单位「米平路 / 米爬升」。

    爬 Δz 要 Δz/vz_max 秒，这段时间能巡航 (Δz/vz_max)×vx_max 米，
    所以爬 1 m 等价于走 vx_max/vz_max 米平路。纯物理，与地图无关。
    """
    if vz_max <= 0:
        raise ValueError('vz_max 必须为正')
    return float(vx_max) / float(vz_max)


def world_to_grid(x, y, origin, res):
    """世界坐标 -> 栅格坐标（向下取整，与 map_projector 的写法一致）。"""
    return (int(math.floor((x - origin[0]) / res)),
            int(math.floor((y - origin[1]) / res)))


def grid_to_world(ix, iy, origin, res):
    """栅格坐标 -> 该格中心的世界坐标。"""
    return (origin[0] + (ix + 0.5) * res, origin[1] + (iy + 0.5) * res)


def clearance_risk(clearance, required=REQUIRED_CLEARANCE):
    """净空风险，0..1。

    clearance >= required 时为 0 —— 裕度足够就**不加惩罚**。
    门槛 `required` 是**安全量**（robot_radius + 实测跟踪偏差），不是偏好量；
    调 α 只改变「愿意为更宽走廊绕多远」，不影响安全判定。
    """
    c = float(clearance)
    if c >= float(required):
        return 0.0
    r = 1.0 - c / float(required)
    return r * r


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


def plan_single_layer(grid, start_ij, goal_ij, origin, res, alpha=2.0,
                      required_clearance=REQUIRED_CLEARANCE,
                      robot_radius=ROBOT_RADIUS):
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
    # 净空：到最近障碍的距离（米）。硬约束 —— 塞不进机体的格直接不可通行。
    if _ndimage is None:
        raise RuntimeError('scipy is required for clearance-aware planning')
    if not blocked.any():
        dist = np.full(g.shape, math.inf)
    else:
        dist = _ndimage.distance_transform_edt(~blocked) * float(res)
    blocked = blocked | (dist < float(robot_radius))

    if not (0 <= sx < w and 0 <= sy < h) or not (0 <= gx < w and 0 <= gy < h):
        return _empty(0.0, 0.0, False)
    if blocked[sy, sx] or blocked[gy, gx]:
        return _empty(0.0, 0.0, False)

    # 代价 = 距离 × (1 + α × 净空风险)。安全由 robot_radius / required_clearance
    # 两个门槛管，α 只表达「为更宽走廊绕远路」的偏好。
    risk = np.vectorize(lambda c: clearance_risk(c, required_clearance))(dist)
    step_cost = 1.0 + float(alpha) * risk

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


class CorridorCache:
    """预计算每个 (格子, 高度区间) 的垂直走廊通畅性。

    原来每次查询都扫整个点云（28 万点），A* 每展开一个节点就查一次 ->
    全分辨率跑不动，只能降采样到 0.3m，而要区分的净空差只有 0.22m。
    这里把点云一次性打成 (ix, iy, iz) 体素，查询退化成对 iz 区间做 any()。
    """

    def __init__(self, points_xyz, origin, res, shape, zs, radius):
        self.res = float(res)
        self.radius = float(radius)
        h, w = shape
        pts = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
        if pts.size:
            pzmin, pzmax = float(pts[:, 2].min()), float(pts[:, 2].max())
        else:                      # 空点云 -> 无障碍，走廊一律通畅
            pzmin = pzmax = 0.0
        self.z0 = float(min(pzmin, min(zs)) - radius - res)
        ztop = float(max(pzmax, max(zs)) + radius + res)
        self.nz = int(math.ceil((ztop - self.z0) / self.res)) + 1
        self.vox = np.zeros((h, w, self.nz), dtype=bool)

        if pts.size:
            ix = np.floor((pts[:, 0] - origin[0]) / self.res).astype(np.int64)
            iy = np.floor((pts[:, 1] - origin[1]) / self.res).astype(np.int64)
            iz = np.floor((pts[:, 2] - self.z0) / self.res).astype(np.int64)
            rr = int(math.ceil(radius / self.res))
            for ddx in range(-rr, rr + 1):
                for ddy in range(-rr, rr + 1):
                    if ddx * ddx + ddy * ddy > rr * rr + rr:
                        continue
                    x2, y2 = ix + ddx, iy + ddy
                    ok = (x2 >= 0) & (x2 < w) & (y2 >= 0) & (y2 < h) &                          (iz >= 0) & (iz < self.nz)
                    self.vox[y2[ok], x2[ok], iz[ok]] = True

    def clear(self, ix, iy, z0, z1):
        lo, hi = (z0, z1) if z0 <= z1 else (z1, z0)
        a = int(math.floor((lo - self.z0) / self.res))
        b = int(math.ceil((hi - self.z0) / self.res)) + 1
        a = max(0, a)
        b = min(self.nz, b)
        if b <= a:
            return True
        return not bool(self.vox[iy, ix, a:b].any())


def _corridor_clear(points_xyz, x, y, z0, z1, radius):
    from marsim_nav.vcorridor import check_vertical_corridor
    return bool(check_vertical_corridor(points_xyz, x, y, z0, z1, radius)["clear"])


def plan_slices(grids, z_centers, points_xyz, start_xyz, goal_xyz, origin, res,
                alpha=DETOUR_PENALTY, beta=None, radius=0.25,
                required_clearance=REQUIRED_CLEARANCE,
                robot_radius=ROBOT_RADIUS):
    """多层 A*：状态 (ix, iy, 层号)，层间移动需垂直走廊通畅。

    - 层内：8 邻接，cost = 距离 × (1 + α × 该格占用)
    - 层间：同格换层，cost = β × |Δz|，**前置条件是该处走廊通畅**

    返回 {"waypoints": [(x,y,z)...], "layer_costs": [...], "reached": bool}。
    waypoints 是世界坐标；相邻路点 z 不同处即升降点。
    """
    if beta is None:
        beta = beta_from_speeds()
    gs = [np.asarray(g) for g in grids]
    if not gs or len(gs) != len(z_centers):
        raise ValueError('grids 与 z_centers 数量必须一致且非空')
    shape = gs[0].shape
    if any(g.shape != shape for g in gs):
        raise ValueError('所有层的栅格尺寸必须一致')
    h, w = shape
    nL = len(gs)
    zs = [float(z) for z in z_centers]
    cache = CorridorCache(points_xyz, origin, res, gs[0].shape, zs, radius)

    sx, sy = world_to_grid(start_xyz[0], start_xyz[1], origin, res)
    gx, gy = world_to_grid(goal_xyz[0], goal_xyz[1], origin, res)
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return {"waypoints": [], "layer_costs": [], "reached": False}

    # 起点/终点的层：最接近其 z 的那一层
    sk = min(range(nL), key=lambda k: abs(zs[k] - float(start_xyz[2])))
    gk = min(range(nL), key=lambda k: abs(zs[k] - float(goal_xyz[2])))

    blocked = [g == 0 for g in gs]           # 0 = 障碍
    if _ndimage is None:
        raise RuntimeError('scipy is required for clearance-aware planning')
    dists = [np.full(b.shape, math.inf) if not b.any()
             else _ndimage.distance_transform_edt(~b) * float(res) for b in blocked]
    blocked = [b | (d < float(robot_radius)) for b, d in zip(blocked, dists)]
    step = [1.0 + float(alpha) * np.vectorize(
        lambda c: clearance_risk(c, required_clearance))(d) for d in dists]

    def hcost(x, y):
        # 只含距离项，故不高估 -> 仍是最优解
        return math.hypot(x - gx, y - gy)

    start, goal = (sx, sy, sk), (gx, gy, gk)
    dist = {start: 0.0}
    prev = {}
    pq = [(hcost(sx, sy), start)]
    seen = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in seen:
            continue
        seen.add(cur)
        if cur == goal:
            break
        cx, cy, ck = cur
        # 层内
        for nx, ny, d in _neighbours(cx, cy, w, h):
            if blocked[ck][ny, nx]:
                continue
            nxt = (nx, ny, ck)
            nd = dist[cur] + d * float(step[ck][ny, nx])
            if nd < dist.get(nxt, math.inf):
                dist[nxt] = nd
                prev[nxt] = cur
                heapq.heappush(pq, (nd + hcost(nx, ny), nxt))
        # 层间：同格换层，需走廊通畅
        wx, wy = grid_to_world(cx, cy, origin, res)
        for nk in range(nL):
            if nk == ck or blocked[nk][cy, cx]:
                continue
            if not cache.clear(cx, cy, zs[ck], zs[nk]):
                continue
            nxt = (cx, cy, nk)
            nd = dist[cur] + abs(zs[nk] - zs[ck]) * float(beta)
            if nd < dist.get(nxt, math.inf):
                dist[nxt] = nd
                prev[nxt] = cur
                heapq.heappush(pq, (nd + hcost(cx, cy), nxt))

    # 每层单独也跑一次，用于 layer_costs（可解释性）
    layer_costs = []
    for k in range(nL):
        single = plan_single_layer(gs[k], (sx, sy), (gx, gy), origin, res, alpha,
                                   required_clearance, robot_radius)
        layer_costs.append({
            "layer": k, "z": zs[k], "reachable": bool(single["reached"]),
            "path_len": float(single["length"]),
            # 不可达的层代价记 inf，不是 0 —— 否则它会排到「最优」
            "cost": float(single["cost"]) if single["reached"] else math.inf,
        })

    if goal not in dist:
        return {"waypoints": [], "layer_costs": layer_costs, "reached": False}

    chain = [goal]
    while chain[-1] != start:
        chain.append(prev[chain[-1]])
    chain.reverse()
    waypoints = [grid_to_world(ix, iy, origin, res) + (zs[k],) for ix, iy, k in chain]
    return {"waypoints": waypoints, "layer_costs": layer_costs, "reached": True}
