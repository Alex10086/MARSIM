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
