"""Diagnose Nav2 navigation problems by measuring, not guessing.

This is the tool that found the two real defects in this integration. Use it
BEFORE tuning anything: several plausible-sounding parameter changes were
measured to have literally zero effect (see README section 12), while the actual
causes were only visible in this data.

It reports, for one goal:

  * plan clearance   -- how far the global plan stays from real obstacles, and
                        how much of the free space it left unused
  * commanded vs achieved speed  -- attributes a "too slow" complaint to Nav2
                        (MPPI commands little) or to the vehicle (it cannot
                        deliver what MPPI commands)
  * cross-track error against the plan CURRENT AT THE TIME  -- never against the
                        first plan, which Nav2 has long since replaced; comparing
                        to a stale plan exaggerates the error badly
  * a time-binned timeline of |cmd|, wz, yaw, heading-vs-motion and the local
                        costmap cost at the vehicle

Usage
-----
    python3 nav_diag.py -28 -12            # full run, default 240 s budget
    python3 nav_diag.py -28 -12 300        # longer budget
    python3 nav_diag.py --plan -28 -12     # plan only, do not fly (~15 s)

`--plan` exists because flying a whole route to learn whether a PLANNER
parameter helped costs minutes; the plan exists within a second of the goal
being accepted.
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node

def _map_dir():
    """Installed share dir if present, else the source tree. The static map is
    needed to compute clearance to real obstacles."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('marsim_nav'), 'maps')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'maps')


MAP_DIR = _map_dir()


def _load_clearance():
    """Distance from every map cell to the nearest real obstacle, in metres."""
    from scipy import ndimage
    my = yaml.safe_load(open(os.path.join(MAP_DIR, 'forest.yaml')))
    res, ox, oy = my['resolution'], my['origin'][0], my['origin'][1]
    parts = open(os.path.join(MAP_DIR, 'forest.pgm'), 'rb').read().split(b'\n', 3)
    w, h = map(int, parts[1].split())
    occ = (np.frombuffer(parts[3][:w * h], dtype=np.uint8).reshape(h, w) == 0)
    return ndimage.distance_transform_edt(~occ) * res, res, ox, oy, w, h


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def cross_track(pt, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = (ab ** 2).sum(1)
    L2[L2 < 1e-12] = 1e-12
    t = np.clip(((pt - a) * ab).sum(1) / L2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.hypot(*(pt - proj).T).min())


class Diag(Node):
    def __init__(self):
        super().__init__('nav_diag')
        self.od = []      # (t, x, y, yaw)
        self.cv = []      # (t, vx, vy, wz)
        self.plan = []    # (t, Nx2)
        self.grid = None
        self.samples = []  # (t, x, y, yaw, cost_at_vehicle)
        self.create_subscription(Odometry, '/odom', self._od, 100)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cv, 100)
        self.create_subscription(Path, '/plan', self._plan, 10)
        # depth 1: only the newest costmap matters
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._grid, 1)

    def _od(self, m):
        p = m.pose.pose.position
        self.od.append((time.time(), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def _cv(self, m):
        self.cv.append((time.time(), m.linear.x, m.linear.y, m.angular.z))

    def _plan(self, m):
        if len(m.poses) > 1:
            self.plan.append((time.time(), np.array(
                [[p.pose.position.x, p.pose.position.y] for p in m.poses])))

    def _grid(self, m):
        self.grid = m

    def cost_at(self, x, y):
        g = self.grid
        if g is None:
            return float('nan')
        c = int((x - g.info.origin.position.x) / g.info.resolution)
        r = g.info.height - 1 - int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= r < g.info.height and 0 <= c < g.info.width):
            return float('nan')
        # OccupancyGrid.data is int8; costmap costs above 127 arrive negative.
        return float(np.frombuffer(g.data, dtype=np.int8).reshape(
            g.info.height, g.info.width)[r, c] & 0xFF)


def report_plan(plan, clearance, res, ox, oy, w, h, label):
    if plan is None or len(plan) < 2:
        print(f'{label}: 无计划')
        return
    P = np.array(plan)

    def rc(x, y):
        return (h - 1) - int((y - oy) / res), int((x - ox) / res)

    v = np.array([clearance[rc(x, y)] for x, y in P])
    k = int(1.5 / res)
    best = np.empty(len(P))
    for i, (x, y) in enumerate(P):
        r, c = rc(x, y)
        best[i] = clearance[max(0, r - k):min(h, r + k + 1),
                            max(0, c - k):min(w, c + k + 1)].max()
    gap = best - v
    print(f'{label}: {len(P)} 点  到最近真实障碍 min={v.min():.2f} '
          f'中位={np.median(v):.2f}  | 邻域(1.5m)可达中位={np.median(best):.2f} '
          f'未用余量 gap中位={np.median(gap):.2f}')
    print(f'    最窄处: 实际 {v.min():.2f}m，该处可达 {best[int(np.argmin(v))]:.2f}m')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gx', type=float)
    ap.add_argument('gy', type=float)
    ap.add_argument('budget', nargs='?', type=float, default=240.0)
    ap.add_argument('--plan', action='store_true',
                    help='only fetch and evaluate the first plan; never fly')
    a = ap.parse_args()

    clearance, res, ox, oy, w, h = _load_clearance()
    rclpy.init()
    n = Diag()
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not ac.wait_for_server(timeout_sec=60):
        print('NO_ACTION_SERVER —— Nav2 没起来，先查 lifecycle')
        return 1

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = a.gx, a.gy
    goal.pose.pose.orientation.w = 1.0

    handle = None
    for attempt in range(12):
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=15)
        handle = fut.result()
        if handle and handle.accepted:
            break
        print(f'  目标被拒（第 {attempt + 1} 次，BT 可能尚未就绪），重试')
        time.sleep(2)
    if not handle or not handle.accepted:
        print('GOAL_ALWAYS_REJECTED')
        return 1
    print(f'目标 ({a.gx}, {a.gy}) 已接受')

    if a.plan:
        t0 = time.time()
        while not n.plan and time.time() - t0 < 25:
            rclpy.spin_once(n, timeout_sec=0.1)
        handle.cancel_goal_async()
        report_plan(n.plan[0][1] if n.plan else None,
                    clearance, res, ox, oy, w, h, '首条计划')
        rclpy.shutdown()
        return 0

    t0 = time.time()
    last = 0.0
    while time.time() - t0 < a.budget:
        rclpy.spin_once(n, timeout_sec=0.02)
        # The local costmap is a ROLLING window: the cost at a position must be
        # sampled while the vehicle is actually there.
        if n.od and time.time() - last > 0.1:
            last = time.time()
            t, x, y, yw = n.od[-1]
            n.samples.append((t - t0, x, y, yw, n.cost_at(x, y)))

    od = np.array(n.od, dtype=float)
    cv = np.array(n.cv, dtype=float)
    sm = np.array(n.samples, dtype=float) if n.samples else np.zeros((0, 5))
    if len(od) < 20 or len(cv) < 20:
        print('样本太少（/odom 或 /cmd_vel_nav 没有数据？）')
        return 1
    od[:, 0] -= od[0, 0]
    cv[:, 0] -= cv[0, 0]
    if len(sm):
        sm[:, 0] -= 0.0

    if n.plan:
        report_plan(n.plan[0][1], clearance, res, ox, oy, w, h, '首条计划')
    else:
        print('首条计划: 没收到 /plan（规划失败？）')

    sp_cmd = np.hypot(cv[:, 1], cv[:, 2])
    dt = np.diff(od[:, 0])
    good = dt > 1e-3
    sp_act = np.hypot(np.diff(od[:, 1]) / np.where(good, dt, 1),
                      np.diff(od[:, 2]) / np.where(good, dt, 1))[good]
    print(f'\n指令速度  中位={np.median(sp_cmd):.2f}  p90={np.percentile(sp_cmd, 90):.2f}')
    if len(sp_act):
        print(f'实际速度  中位={np.median(sp_act):.2f}  p90={np.percentile(sp_act, 90):.2f}')
    print(f'  指令 <0.02 的占比 {np.mean(sp_cmd < 0.02) * 100:.1f}%'
          f'   ← 高说明 MPPI 在原地犹豫')

    if n.plan and len(od) > 5:
        pt = np.array([p[0] for p in n.plan])
        errs = []
        for t, x, y, _ in od:
            i = np.searchsorted(pt, t, side='right') - 1
            if i >= 0:
                errs.append(cross_track(np.array([x, y]), n.plan[i][1]))
        if errs:
            e = np.array(errs)
            print(f'横向偏差（相对当刻计划）中位={np.median(e):.2f} '
                  f'p90={np.percentile(e, 90):.2f} max={e.max():.2f} m')
            print(f'  >0.25m 占 {np.mean(e > 0.25) * 100:.1f}%')

    trk = od[:, 1:3]
    if len(trk) > 1:
        clr = np.array([clearance[(h - 1) - int((y - oy) / res),
                                  int((x - ox) / res)] for x, y in trk])
        print(f'轨迹到最近真实障碍 min={clr.min():.2f} '
              f'中位={np.median(clr):.2f}  '
              f'低于 robot_radius 的点 {int((clr < 0.25).sum())}/{len(clr)}')

    print(f'\n{"time":>11} {"|cmd|":>6} {"wz":>7} {"yaw°":>7} {"Δyaw°":>7} '
          f'{"cost":>6} {"path(m)":>8}')
    for k in range(int(a.budget // 20)):
        lo, hi = k * 20, (k + 1) * 20
        mc = (cv[:, 0] >= lo) & (cv[:, 0] < hi)
        mo = (od[:, 0] >= lo) & (od[:, 0] < hi)
        if mc.sum() < 5 or mo.sum() < 5:
            continue
        p = od[mo]
        d = float(np.hypot(np.diff(p[:, 1]), np.diff(p[:, 2])).sum())
        hd = math.degrees(math.atan2(p[-1, 2] - p[0, 2], p[-1, 1] - p[0, 1]))
        dy = (math.degrees(p[-1, 3]) - hd + 180) % 360 - 180
        ms = (sm[:, 0] >= lo) & (sm[:, 0] < hi) if len(sm) else None
        cost = (np.nanmedian(sm[ms, 4]) if (ms is not None and ms.sum())
                else float('nan'))
        print(f'{lo:4d}-{hi:4d} {np.median(np.hypot(cv[mc, 1], cv[mc, 2])):>6.2f} '
              f'{np.median(cv[mc, 3]):>7.2f} {math.degrees(p[-1, 3]):>7.1f} '
              f'{dy:>7.1f} {cost:>6.0f} {d:>8.1f}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
