"""One-shot: consume /map_generator/global_cloud and write maps/<name>.pgm/.yaml.

Run ONCE to build the static map, then stop. It walks every point, so it is not
meant to run alongside real-time navigation.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from marsim_nav.map_projector import project, write_pgm, write_yaml


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        self.declare_parameter('cloud_topic', '/map_generator/global_cloud')
        self.declare_parameter('output_dir', os.path.expanduser('~/marsim_maps'))
        self.declare_parameter('output_name', 'forest')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('margin', 2.0)
        self.declare_parameter('inflation_radius', 0.4)
        self._done = False
        topic = self.get_parameter('cloud_topic').value
        self.create_subscription(PointCloud2, topic, self._cb, 1)
        self.get_logger().info(f'occupancy_grid_node waiting on {topic}')

    def _cb(self, msg: PointCloud2):
        if self._done:
            return
        self._done = True
        rec = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in rec], dtype=np.float64)
        r = project(pts,
                    res=self.get_parameter('resolution').value,
                    z_min=self.get_parameter('z_min').value,
                    z_max=self.get_parameter('z_max').value,
                    margin=self.get_parameter('margin').value,
                    inflation_radius=self.get_parameter('inflation_radius').value)
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        name = self.get_parameter('output_name').value
        pgm = os.path.join(out_dir, name + '.pgm')
        yml = os.path.join(out_dir, name + '.yaml')
        write_pgm(pgm, r.grid)
        write_yaml(yml, os.path.basename(pgm), r.resolution, r.origin)
        self.get_logger().info(
            f'wrote {pgm} ({r.grid.shape[1]}x{r.grid.shape[0]}) and {yml}; '
            f'origin={r.origin}')


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
