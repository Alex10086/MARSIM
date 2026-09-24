#!/usr/bin/env python3
"""Inject geometry_msgs/Twist commands into quad_pid, without Nav2.

Purpose: decouple two failure domains. When a Nav2 end-to-end run misbehaves you
cannot tell whether the controller or the Nav2 configuration is at fault. This
tool drives /cmd_vel directly, so running it against the same controller answers
that question in one shot.

Twist contract (see quad_pid/modes.py):
  linear.x / linear.y  -- BODY-frame velocity in m/s (rotated by current yaw)
  angular.z            -- yaw RATE in rad/s (a rate, NOT an angle)
  linear.z             -- ignored unless twist_follow_z is set

By default the command STOPS being published after --duration. That is the
interesting case: a velocity controller must freeze its setpoint and brake, never
keep integrating the last command into a runaway.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def opt_float(s):
    """argparse type that maps an empty string to None.

    The launch file always passes --yaw, so that `yaw:=` on the command line
    means "do not pre-rotate" rather than float('') blowing up.
    """
    s = (s or '').strip()
    return None if s == '' else float(s)


class TwistInjector(Node):
    def __init__(self, args):
        super().__init__('twist_pub')
        self.args = args
        self.pos = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.have_odom = False

        # RELIABLE depth 10 matches nav2 controller_server's
        # rclcpp::SystemDefaultsQoS(), so this also exercises the QoS
        # compatibility claim against our BEST_EFFORT subscriber.
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', pub_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/cmd', pub_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sub_qos)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos = [p.x, p.y, p.z]
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.have_odom = True

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def pre_rotate(self, target_deg):
        """Rotate in place to the target yaw using the position path.

        Sends a position goal at the CURRENT position with the desired yaw --
        exactly what an RViz drag does -- then waits for the yaw to settle. This
        is what makes the yaw!=0 straight-line case (the orbit regression) a real
        test rather than an assumption about what the drone's yaw happens to be.
        """
        target = math.radians(target_deg)
        self.spin_for(1.0)
        if not self.have_odom:
            raise RuntimeError('no /odom received; cannot pre-rotate')

        m = PoseStamped()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.pos
        m.pose.orientation.z = math.sin(target / 2.0)
        m.pose.orientation.w = math.cos(target / 2.0)

        tol = math.radians(self.args.yaw_tol)
        t0 = time.time()
        while time.time() - t0 < 20.0 and rclpy.ok():
            m.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(m)
            self.spin_for(0.1)
            err = abs(math.atan2(math.sin(self.yaw - target),
                                 math.cos(self.yaw - target)))
            if err < tol:
                self.get_logger().info(
                    f'pre-rotate done: yaw={math.degrees(self.yaw):.1f} deg '
                    f'(target {target_deg:.0f}, err {math.degrees(err):.1f})')
                return
        self.get_logger().warn(
            f'pre-rotate did not settle: yaw={math.degrees(self.yaw):.1f} '
            f'target={target_deg:.0f}')

    def command(self, elapsed, duration):
        """Return (vx, vy, wz) in the BODY frame for the given elapsed time."""
        a = self.args
        if a.pattern == 'zero':
            return (0.0, 0.0, 0.0)
        if a.pattern == 'forward':
            return (a.vx, 0.0, 0.0)
        if a.pattern == 'backward':
            return (-a.vx, 0.0, 0.0)
        if a.pattern == 'strafe':
            return (0.0, a.vy, 0.0)
        if a.pattern == 'spin':
            return (0.0, 0.0, a.wz)
        if a.pattern == 'square':
            # Four equal body-frame legs: +x, +y, -x, -y. At yaw=0 this traces
            # a square in the world; it exercises all four velocity directions.
            leg = max(duration, 1e-6) / 4.0
            k = min(int(elapsed / leg), 3)
            return [(a.vx, 0.0, 0.0), (0.0, a.vy, 0.0),
                    (-a.vx, 0.0, 0.0), (0.0, -a.vy, 0.0)][k]
        raise ValueError(f'unknown pattern {a.pattern}')

    def run(self):
        a = self.args
        if a.yaw is not None:
            self.pre_rotate(a.yaw)

        self.get_logger().info(
            f'pattern={a.pattern} vx={a.vx} vy={a.vy} wz={a.wz} '
            f'duration={a.duration}s -> will STOP publishing afterwards')

        t0 = time.time()
        last_log = 0.0
        while rclpy.ok():
            elapsed = time.time() - t0
            if elapsed >= a.duration:
                break
            vx, vy, wz = self.command(elapsed, a.duration)
            m = Twist()
            m.linear.x, m.linear.y, m.linear.z = vx, vy, 0.0
            m.angular.z = wz
            self.twist_pub.publish(m)
            if elapsed - last_log >= 1.0:
                last_log = elapsed
                self.get_logger().info(
                    f't={elapsed:4.1f}s cmd=({vx:.2f},{vy:.2f},{wz:.2f}) '
                    f'pos=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f}) '
                    f'yaw={math.degrees(self.yaw):6.1f}')
            self.spin_for(1.0 / a.rate)

        self.get_logger().info('STOPPED publishing /cmd_vel (observing freeze)')
        # Linger with no publishing: the controller must brake and hold.
        t1 = time.time()
        p0 = list(self.pos)
        while rclpy.ok() and time.time() - t1 < a.linger:
            self.spin_for(0.2)
        drift = math.dist(self.pos[:2], p0[:2])
        self.get_logger().info(
            f'final pos=({self.pos[0]:.3f},{self.pos[1]:.3f},{self.pos[2]:.3f}) '
            f'yaw={math.degrees(self.yaw):.1f} '
            f'drift_after_stop={drift:.3f}m over {a.linger:.0f}s')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pattern', required=True,
                    choices=['zero', 'forward', 'backward', 'strafe', 'spin', 'square'])
    ap.add_argument('--vx', type=float, default=1.0)
    ap.add_argument('--vy', type=float, default=1.0)
    ap.add_argument('--wz', type=float, default=0.5)
    ap.add_argument('--duration', type=float, default=5.0)
    ap.add_argument('--linger', type=float, default=6.0,
                    help='seconds to stay alive after publishing stops')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--yaw', type=opt_float, default=None,
                    help='pre-rotate to this yaw (deg) before injecting; '
                         'empty/omitted = no pre-rotation')
    ap.add_argument('--yaw-tol', type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = TwistInjector(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
