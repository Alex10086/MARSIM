"""把 Nav2 的平面速度指令与高度剖面的垂直速度合成一条 Twist。

纯逻辑不 import rclpy，节点只是薄壳。

为什么需要它：Nav2 的 MPPI `Omni` 是平面模型，`linear.z` 恒为 0，
所以没有任何人告诉无人机该爬。这里把旁路的高度指令并进去。
"""
import math


def _finite(v):
    return float(v) if math.isfinite(v) else 0.0


def merge_twist(nav_vx, nav_vy, nav_wz, alt_vz, alt_age, timeout=0.5):
    """返回 (vx, vy, vz, wz)。

    - 非有限输入一律置 0.0（与 quad_pid.clamp_twist 的既有约定一致：
      那里的 min/max 会把 NaN 变成满量程指令，所以必须在这里挡掉）
    - `alt_age >= timeout` 时 vz 归零（超时含端点：与 quad_pid 的
      cmd_vel_timeout 语义保持一致，避免"恰好超时"仍被当作新鲜）
    """
    vx, vy, wz = _finite(nav_vx), _finite(nav_vy), _finite(nav_wz)
    vz = _finite(alt_vz) if alt_age < timeout else 0.0
    return (vx, vy, vz, wz)


class AltCmdState:
    """记录最近一次 /alt_cmd 的值与时刻；从未收到时 age 为 inf。"""

    def __init__(self):
        self.vz = 0.0
        self.stamp = None

    def update(self, vz, stamp):
        self.vz = float(vz)
        self.stamp = float(stamp)

    def age(self, now):
        return math.inf if self.stamp is None else float(now) - self.stamp


def main(args=None):
    import time

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float32

    class MergeCmdVelNode(Node):
        def __init__(self):
            super().__init__('merge_cmd_vel_node')
            self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
            self.declare_parameter('alt_cmd_topic', '/alt_cmd')
            self.declare_parameter('out_topic', '/cmd_vel_merged')
            self.declare_parameter('alt_timeout', 0.5)
            self._alt = AltCmdState()
            self._nav = Twist()
            self._pub = self.create_publisher(
                Twist, self.get_parameter('out_topic').value, 10)
            self.create_subscription(
                Twist, self.get_parameter('cmd_vel_nav_topic').value,
                self._on_nav, 10)
            self.create_subscription(
                Float32, self.get_parameter('alt_cmd_topic').value,
                self._on_alt, 10)
            self.create_timer(0.05, self._on_tick)   # 20 Hz，与 Nav2 一致

        def _on_nav(self, msg):
            self._nav = msg

        def _on_alt(self, msg):
            self._alt.update(msg.data, time.time())

        def _on_tick(self):
            n = self._nav
            vx, vy, vz, wz = merge_twist(
                n.linear.x, n.linear.y, n.angular.z,
                self._alt.vz, self._alt.age(time.time()),
                timeout=self.get_parameter('alt_timeout').value)
            out = Twist()
            out.linear.x, out.linear.y, out.linear.z = vx, vy, vz
            out.angular.z = wz
            self._pub.publish(out)

    rclpy.init(args=args)
    node = MergeCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
