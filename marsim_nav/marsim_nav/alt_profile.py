"""按脚本化剖面发布 /alt_cmd，驱动中途升降。

每个爬升/降落段之前先做垂直走廊检查（spec §4.2/§4.3）；不通过则**保持当前
高度**并记录最近的通畅柱子位置，由 Nav2 重规划或人工横移后再执行。
第二步的切片规划器从源头只选走廊通畅的位置作为爬升点。
"""
import math

import yaml


def next_vz(z_current, z_target, kp=0.8, vz_max=0.6, arrive_tol=0.15):
    """给定当前与目标高度，返回该发的 vz（正值爬升）。"""
    err = float(z_target) - float(z_current)
    if abs(err) <= arrive_tol:
        return 0.0
    return max(-vz_max, min(vz_max, kp * err))


def load_profile(path):
    """读取剖面 YAML，返回 [{x, y, z, note}, ...]；缺 x/y/z 即报错。"""
    with open(path, encoding='utf8') as fh:
        doc = yaml.safe_load(fh) or {}
    segs = doc.get('segments') or []
    out = []
    for i, s in enumerate(segs):
        if not isinstance(s, dict) or not all(k in s for k in ('x', 'y', 'z')):
            raise ValueError(f'segments[{i}] 必须含 x/y/z，实际为 {s!r}')
        out.append({'x': float(s['x']), 'y': float(s['y']), 'z': float(s['z']),
                    'note': s.get('note', '')})
    return out


def main(args=None):
    import time

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32

    from marsim_nav.pcd_io import read_pcd
    from marsim_nav.vcorridor import check_vertical_corridor

    class AltProfileNode(Node):
        def __init__(self):
            super().__init__('alt_profile_node')
            self.declare_parameter('profile', '')
            self.declare_parameter('arrive_tol', 0.15)
            self.declare_parameter('kp', 0.8)
            self.declare_parameter('vz_max', 0.6)
            self.declare_parameter('check_radius', 0.25)
            self.declare_parameter('map_pcd', '')
            self._segs = load_profile(self.get_parameter('profile').value)
            self._idx = 0
            self._z = None
            self._checked = set()
            self._pts = read_pcd(self.get_parameter('map_pcd').value)
            self._pub = self.create_publisher(Float32, '/alt_cmd', 10)
            self.create_subscription(Odometry, '/odom', self._on_odom, 10)
            self.create_timer(0.1, self._on_tick)   # 10 Hz

        def _on_odom(self, msg):
            self._z = msg.pose.pose.position.z

        def _on_tick(self):
            if self._z is None or self._idx >= len(self._segs):
                self._pub.publish(Float32(data=0.0))
                return
            seg = self._segs[self._idx]
            # 爬升/降落前查垂直走廊。不通过则保持高度并记录最近的通畅柱子，
            # 由 Nav2 重规划或人工横移后再执行。
            if seg['z'] != self._z and self._idx not in self._checked:
                r = check_vertical_corridor(
                    self._pts, seg['x'], seg['y'], self._z, seg['z'],
                    self.get_parameter('check_radius').value)
                self._checked.add(self._idx)
                if not r['clear']:
                    self.get_logger().warn(
                        f'垂直走廊受阻 z={self._z:.2f}->{seg["z"]:.2f} '
                        f'blocking={r["blocking_z"]} 保持高度，待横移后再执行')
                    self._pub.publish(Float32(data=0.0))
                    return
                self.get_logger().info(
                    f'垂直走廊通畅 z={self._z:.2f}->{seg["z"]:.2f} '
                    f'nearest={r["nearest"]:.2f}')
            vz = next_vz(self._z, seg['z'],
                         self.get_parameter('kp').value,
                         self.get_parameter('vz_max').value,
                         self.get_parameter('arrive_tol').value)
            self._pub.publish(Float32(data=float(vz)))
            if vz == 0.0:
                self._idx += 1

    rclpy.init(args=args)
    node = AltProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
