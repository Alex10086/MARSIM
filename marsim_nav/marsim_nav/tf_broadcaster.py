"""Publish the TF tree Nav2 requires, which MARSIM does not provide.

Edges:
    map       --static identity--> odom
    odom      --static identity--> world        (makes world-frame clouds reachable)
    odom      --dynamic, from /odom--> base_link
    base_link --static--> lidar_link

`odom -> world` is the edge that lets the costmap (global frame map/odom)
consume `/cloud`, which MARSIM stamps `world`. Without it the costmap silently
never marks an obstacle.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from marsim_nav import frames


def _identity(parent, child):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.rotation.w = 1.0
    return t


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('marsim_tf_broadcaster')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('lidar_offset_z', 0.1)

        self._static_bc = StaticTransformBroadcaster(self)
        named = [
            _identity(frames.MAP, frames.ODOM),
            _identity(frames.ODOM, frames.WORLD),
        ]
        lidar = _identity(frames.BASE, frames.LIDAR)
        lidar.transform.translation.z = float(
            self.get_parameter('lidar_offset_z').value)
        named.append(lidar)
        self._static_bc.sendTransform(named)

        self._dyn_bc = TransformBroadcaster(self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.get_logger().info(
            f'marsim_tf_broadcaster up: map->odom, odom->world, '
            f'base_link->lidar_link, and odom->base_link from {odom_topic}')

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = frames.ODOM
        t.child_frame_id = frames.BASE
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._dyn_bc.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
