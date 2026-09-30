"""发一个三维点 (x, y, z) 并让无人机到达。

把三块已有的东西接起来，不新增算法：
  (x, y)  -> Nav2 的 navigate_to_pose 动作
  z       -> /alt_cmd（复用 alt_profile 的 next_vz / commanded_vz / segment_done）
  安全    -> vcorridor 垂直走廊检查（爬升前）

用法（支持多个点，逐个访问）：
    ros2 run marsim_nav goto3d -- <x> <y> <z> [<x> <y> <z> ...] [秒]
"""
import math


def goal_reached_3d(cur_xyz, target_xyz, xy_tol=0.35, z_tol=0.3):
    """三维到达判定：水平与高度分别看容差，**两者都要满足**。

    这正是「发三维点」区别于 Nav2 的地方 —— Nav2 只判 (x,y)，
    高度到了没有它不管。
    """
    d = math.hypot(float(cur_xyz[0]) - float(target_xyz[0]),
                   float(cur_xyz[1]) - float(target_xyz[1]))
    dz = abs(float(cur_xyz[2]) - float(target_xyz[2]))
    # 1e-9 的浮点余量：1.3 - 1.0 == 0.30000000000000004，严格 <= 会误判边界
    eps = 1e-9
    return d <= float(xy_tol) + eps and dz <= float(z_tol) + eps


def main(args=None):
    import sys
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import Float32

    from marsim_nav.alt_profile import commanded_vz, needs_corridor_check
    from marsim_nav.pcd_io import read_pcd
    from marsim_nav.vcorridor import check_vertical_corridor

    argv = sys.argv[1:] if args is None else list(args)
    if len(argv) < 3:
        print(main.__doc__)
        return 1
    # 前 3N 个是路点；若末尾只剩 1 个则当作总时长
    nums = [float(v) for v in argv]
    if len(nums) % 3 == 1:
        duration = nums.pop()
    else:
        duration = 600.0
    if len(nums) % 3 != 0 or not nums:
        print(main.__doc__)
        return 1
    wps = [tuple(nums[i:i + 3]) for i in range(0, len(nums), 3)]
    gx, gy, gz = wps[0]

    class Goto3D(Node):
        def __init__(self):
            super().__init__('goto3d_node')
            self.declare_parameter('check_radius', 0.25)
            self.declare_parameter('vz_max', 0.6)
            self.declare_parameter('kp', 0.8)
            self.declare_parameter('map_pcd', '')
            self._pos = None
            self._gh = None
            self._blocked = False
            self._cleared = False
            self._last_check = None
            self._pts = read_pcd(self.get_parameter('map_pcd').value) \
                if self.get_parameter('map_pcd').value else None
            self._pub = self.create_publisher(Float32, '/alt_cmd', 10)
            self.create_subscription(Odometry, '/odom', self._on_odom, 10)
            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.create_timer(0.1, self._on_tick)

        def send_goal(self, target):
            # 必须先取消上一个动作，否则新目标会被抢占/丢失 ——
            # 实测多点回归里第 3 个点的水平腿就是这么丢的。
            if getattr(self, '_gh', None) is not None:
                try:
                    self._gh.cancel_goal_async()
                except Exception:
                    pass
                self._gh = None
            gx, gy, _ = target
            g = NavigateToPose.Goal()
            g.pose = PoseStamped()
            g.pose.header.frame_id = 'map'
            g.pose.pose.position.x = gx
            g.pose.pose.position.y = gy
            g.pose.pose.orientation.w = 1.0
            fut = self.client.send_goal_async(g)

            def _on_accepted(f):
                try:
                    self._gh = f.result()
                except Exception:
                    self._gh = None
            fut.add_done_callback(_on_accepted)
            self.get_logger().info(f'发 Nav2 目标 ({gx:.2f},{gy:.2f})，z -> {target[2]:.2f}')

        def _on_odom(self, m):
            p = m.pose.pose.position
            self._pos = (p.x, p.y, p.z)

        def _on_tick(self):
            if self._pos is None:
                return
            x, y, z = self._pos
            # 爬升前查垂直走廊（不通过则保持高度，与 alt_profile 同一语义）
            tgt_z = wps[0][2] if wps else gz
            if self._pts is not None and not self._cleared:
                if needs_corridor_check(0, z, tgt_z, {0} if self._cleared else set(),
                                        self._last_check, time.time()):
                    self._last_check = time.time()
                    r = check_vertical_corridor(self._pts, x, y, z, tgt_z,
                                                self.get_parameter('check_radius').value)
                    if not r['clear']:
                        self._blocked = True
                        self.get_logger().warn(
                            f'垂直走廊受阻 z={z:.2f}->{tgt_z:.2f} '
                            f'blocking={r["blocking_z"]} 保持高度')
                    else:
                        self._blocked = False
                        self._cleared = True
                        self.get_logger().info(
                            f'垂直走廊通畅 z={z:.2f}->{tgt_z:.2f} nearest={r["nearest"]:.2f}')
            tgt_z = wps[0][2] if wps else gz
            vz = commanded_vz(self._blocked, z, tgt_z,
                              self.get_parameter('kp').value,
                              self.get_parameter('vz_max').value)
            self._pub.publish(Float32(data=float(vz)))
            if wps and goal_reached_3d(self._pos, wps[0]):
                x2, y2, z2 = wps.pop(0)
                self.get_logger().info(
                    f'已到达 ({x2:.2f},{y2:.2f},{z2:.2f})，实际 '
                    f'({x:.2f},{y:.2f},{z:.2f})  剩余 {len(wps)} 个')
                if wps:
                    self._cleared = False
                    self._blocked = False
                    self._last_check = None
                    self.send_goal(wps[0])

    wps0 = list(wps)
    rclpy.init(args=args)
    n = Goto3D()
    t0 = time.time()
    sent = False
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(n, timeout_sec=0.2)
            if not sent and n.client.wait_for_server(timeout_sec=1.0):
                n.send_goal(wps[0]); sent = True
            if n._pos and not wps:
                break
        print(f'结果: 全部 {len(wps0)} 个路点，剩余未到 {len(wps)} 个', flush=True)
        if n._pos:
            x, y, z = n._pos
            print(f'终点: 实际 ({x:.2f},{y:.2f},{z:.2f})', flush=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
