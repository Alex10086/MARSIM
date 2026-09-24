"""Watch a navigation goal live, printing every second so an interrupted run
still leaves data behind.

Every probe in this directory that buffered its output and printed a report at
the end was useless the moment a run was interrupted -- which, while debugging
something interactive, is most of the time. This one prints one flushed line per
second, so `tail` shows the whole history whatever happens.

Each line answers one question:

  status   ACCEPTED/EXECUTING/SUCCEEDED/ABORTED -- is the action still running?
           (EXECUTING while distance is small => something is blocking the goal
            checker; nothing pending while RViz says "reached" => the BT is stuck)
  dist     distance to the goal
  yaw      current heading (the vehicle's yaw loop does not hold heading well)
  |cmd|    commanded horizontal speed from /cmd_vel_nav -- is MPPI driving?
  wz       commanded yaw rate
  hz       rate of /cmd_vel_nav -- 0 means the controller stopped publishing
  cost     local costmap cost under the vehicle (255 = unknown, 254 = lethal)

Usage:
    python3 live_state.py <gx> <gy> [seconds]      # default 240 s
Interrupt any time with Ctrl-C; what has been printed is the record.
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def emit(line):
    """Print and flush: an interrupted run must still have the data."""
    print(line, flush=True)


def along_plan(plan, pos, dists):
    """Points on `plan` at given arc-length offsets AHEAD of `pos`.

    The cost at the vehicle itself says nothing about why MPPI slows down: MPPI
    scores whole trajectories over a ~2.8 s horizon, so what matters is the cost
    it sees in FRONT of the vehicle. Sampling ALONG THE PLAN (not a straight
    line) is what makes the measurement comparable to what the controller is
    actually trying to follow.
    """
    P = np.asarray(plan, dtype=float)
    if len(P) < 2:
        return [None] * len(dists)
    seg = np.maximum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])), 1e-9)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    a, b = P[:-1], P[1:]
    ab = b - a
    L2 = (ab ** 2).sum(1)
    t = np.clip(((np.asarray(pos) - a) * ab).sum(1) / np.maximum(L2, 1e-12), 0, 1)
    proj = a + t[:, None] * ab
    i = int(np.argmin(np.hypot(pos[0] - proj[:, 0], pos[1] - proj[:, 1])))
    s0 = s[i] + t[i] * seg[i]
    out = []
    for d in dists:
        sd = s0 + d
        if sd >= s[-1]:
            out.append(None)
            continue
        j = max(0, min(int(np.searchsorted(s, sd) - 1), len(P) - 2))
        out.append(P[j] + ((sd - s[j]) / seg[j]) * (P[j + 1] - P[j]))
    return out


class Live(Node):
    def __init__(self):
        super().__init__('live_state')
        self.pos = None
        self.yaw = None
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_n = 0
        self.status = None
        self.grid = None
        self.plan_n = 0
        self.plan = None
        self.create_subscription(Odometry, '/odom', self._od, 50)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cv, 50)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._grid, 1)
        self.create_subscription(Path, '/plan', self._plan, 10)
        from action_msgs.msg import GoalStatusArray
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self._st, 10)

    def _od(self, m):
        p = m.pose.pose.position
        self.pos = (p.x, p.y)
        self.yaw = yaw_of(m.pose.pose.orientation)

    def _cv(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_n += 1

    def _st(self, m):
        if m.status_list:
            self.status = m.status_list[-1].status

    def _plan(self, m):
        self.plan_n += 1
        if len(m.poses) > 1:
            self.plan = np.array([[p.pose.position.x, p.pose.position.y]
                                  for p in m.poses])

    def _grid(self, m):
        self.grid = m

    def cost_at(self, x, y):
        g = self.grid
        if g is None or x is None:
            return float('nan')
        c = int((x - g.info.origin.position.x) / g.info.resolution)
        r = g.info.height - 1 - int((y - g.info.origin.position.y)
                                    / g.info.resolution)
        if not (0 <= r < g.info.height and 0 <= c < g.info.width):
            return float('nan')
        # OccupancyGrid.data is int8, and the costmap PUBLISHER writes
        # LETHAL_OBSTACLE (254) as 100 and INSCRIBED (253) as 99 -- so on these
        # topics 100 means blocked, and counting ==254 always yields zero.
        return float(np.frombuffer(g.data, dtype=np.int8).reshape(
            g.info.height, g.info.width)[r, c] & 0xFF)

    def cost_here(self):
        return self.cost_at(*(self.pos if self.pos else (None, None)))

    def ahead_costs(self, dists=(1.0, 2.0, 3.0, 5.0)):
        """Costmap cost at 1/2/3/5 m ahead ALONG THE PLAN."""
        if self.plan is None or self.pos is None:
            return [float('nan')] * len(dists)
        pts = along_plan(self.plan, self.pos, dists)
        return [self.cost_at(*p) if p is not None else float('nan') for p in pts]


def main():
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 240.0
    rclpy.init()
    n = Live()
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not ac.wait_for_server(timeout_sec=60):
        emit('NO_ACTION_SERVER —— Nav2 没起来，先查 lifecycle')
        return 1
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.w = 1.0
    h = None
    for a in range(12):
        f = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, f, timeout_sec=15)
        h = f.result()
        if h and h.accepted:
            break
        emit(f'  目标被拒（第 {a + 1} 次，BT 可能未就绪），重试')
        time.sleep(2)
    if not h or not h.accepted:
        emit('GOAL_ALWAYS_REJECTED')
        return 1

    emit(f'目标 ({gx}, {gy}) 已接受，最多记录 {secs:.0f}s（随时 Ctrl-C）')
    emit('cost 列 = 机体处 / 沿当前计划前方 1m / 2m / 3m / 5m')
    emit(f'{"t":>6} {"status":>10} {"dist":>6} {"|cmd|":>6} {"wz":>7} '
         f'{"@0":>5} {"+1m":>5} {"+2m":>5} {"+3m":>5} {"+5m":>5}')
    res = h.get_result_async()
    t0 = time.time()
    nxt = t0
    try:
        while time.time() - t0 < secs:
            rclpy.spin_once(n, timeout_sec=0.05)
            now = time.time()
            if now >= nxt:
                nxt = now + 1.0
                d = (math.hypot(n.pos[0] - gx, n.pos[1] - gy)
                     if n.pos else float('nan'))
                ch, c1, c2, c3, c5 = n.cost_here(), *n.ahead_costs()
                emit(f'{now - t0:>6.1f} {STATUS.get(n.status, "?"):>10} '
                     f'{d:>6.2f} '
                     f'{math.hypot(n.cmd[0], n.cmd[1]):>6.2f} {n.cmd[2]:>7.2f} '
                     f'{ch:>5.0f} {c1:>5.0f} {c2:>5.0f} {c3:>5.0f} {c5:>5.0f}')
            if res.done():
                emit(f'*** 动作在 {time.time() - t0:.1f}s 结束：'
                     f'{STATUS.get(res.result().status, "?")} ***')
                break
        else:
            emit(f'*** {secs:.0f}s 内动作【没有】结束，'
                 f'当前 status={STATUS.get(n.status, "?")} ***')
    except KeyboardInterrupt:
        emit(f'*** 被中断于 {time.time() - t0:.1f}s '
             f'(status={STATUS.get(n.status, "?")}) ***')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
