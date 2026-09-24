"""Republish MARSIM's world-frame `/cloud` in the LiDAR frame for Nav2.

See `marsim_nav.cloud_transform` for why this is necessary. The original
`/cloud` is left untouched, so any other consumer keeps working; this node adds
`/cloud_lidar` (xyz-only, `frame_id: lidar_link`) for the costmap obstacle
layer.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from marsim_nav import frames
from marsim_nav.cloud_transform import world_to_lidar


def _quat_to_matrix(x, y, z, w):
    """Body-to-world rotation from a quaternion. No tf2 needed for one 3x3."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def _xyz_offsets(msg):
    off = {f.name: f.offset for f in msg.fields}
    missing = [n for n in ('x', 'y', 'z') if n not in off]
    if missing:
        raise ValueError(f'cloud is missing fields {missing}')
    return off['x'], off['y'], off['z']


def extract_xyz(msg):
    """Pull (N, 3) float32 xyz out of a PointCloud2 without laser_geometry."""
    ox, oy, oz = _xyz_offsets(msg)
    n = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    axes = [arr[:, o:o + 4].copy().view(np.float32).reshape(-1) for o in (ox, oy, oz)]
    return np.stack(axes, axis=1)


def build_xyz_cloud(points, header):
    """A minimal, unorganised xyz PointCloud2. NaN points are kept (is_dense False)."""
    pts = np.ascontiguousarray(points, dtype=np.float32)
    out = PointCloud2()
    out.header = header
    out.height = 1
    out.width = pts.shape[0]
    out.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    out.point_step = 12
    out.row_step = 12 * pts.shape[0]
    out.is_bigendian = False
    out.is_dense = False
    out.data = pts.tobytes()
    return out


class CloudReframe(Node):
    def __init__(self):
        super().__init__('marsim_cloud_reframe')
        self.declare_parameter('cloud_in', '/cloud')
        self.declare_parameter('cloud_out', '/cloud_lidar')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('lidar_offset_z', 0.1)

        self._offset = np.array(
            [0.0, 0.0, float(self.get_parameter('lidar_offset_z').value)])
        self._rotation = None
        self._translation = None
        self._dropped = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('cloud_out').value, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, qos)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_in').value, self._cloud_cb, qos)
        self.get_logger().info(
            f"marsim_cloud_reframe up: {self.get_parameter('cloud_in').value} "
            f"(frame '{frames.MARSIM_CLOUD_FRAME}') -> "
            f"{self.get_parameter('cloud_out').value} (frame '{frames.LIDAR}')")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._translation = np.array([p.x, p.y, p.z])
        self._rotation = _quat_to_matrix(q.x, q.y, q.z, q.w)

    def _cloud_cb(self, msg: PointCloud2):
        if self._translation is None:
            # No pose yet: dropping is correct. Publishing world-frame points
            # under the lidar frame would be worse than a gap.
            self._dropped += 1
            if self._dropped == 1:
                self.get_logger().warn('no /odom yet; dropping clouds until it arrives')
            return
        pts = extract_xyz(msg).astype(np.float64)
        lidar = world_to_lidar(pts, self._rotation, self._translation, self._offset)
        hdr = Header()
        hdr.stamp = msg.header.stamp
        hdr.frame_id = frames.LIDAR
        self._pub.publish(build_xyz_cloud(lidar, hdr))


def main(args=None):
    rclpy.init(args=args)
    node = CloudReframe()
    try:
        rclpy.spin(node)
    # SIGINT makes rclpy.spin raise ExternalShutdownException, not
    # KeyboardInterrupt; see tf_broadcaster.py for the full reasoning.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
