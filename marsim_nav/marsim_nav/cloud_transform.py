"""Pure geometry for re-expressing MARSIM's world-frame cloud in the LiDAR frame.

MARSIM stamps `/cloud` with `frame_id: "world"` and writes points already in
world (ENU) coordinates. Nav2's costmap obstacle layer assumes a PointCloud2
lives in the SENSOR frame: it takes the cloud frame's ORIGIN as the sensor
origin and raytrace-clears along rays from there. With a world-frame cloud that
origin is (0, 0), so every clearing ray is cast from the map origin instead of
from the vehicle. Marking still lands in the right cells (the points are real
world coordinates) but nothing near the vehicle is ever cleared, which shows up
as this warning thousands of times per run:

    Sensor origin at (0.00, 0.00) is out of map bounds (38.90, -15.20) to
    (48.85, -5.25). The costmap cannot raytrace for it.

Re-expressing the points into `lidar_link` makes the frame origin the LiDAR's
actual position, so both marking and clearing become correct.
"""
import numpy as np


def world_to_lidar(points, rotation, translation, lidar_offset):
    """Map world-frame points into the LiDAR frame.

    Parameters
    ----------
    points : (N, 3) array_like
        Points in the world frame.
    rotation : (3, 3) array_like
        Body-to-world rotation of the vehicle, i.e. the rotation part of the
        `map -> base_link` transform.
    translation : (3,) array_like
        Vehicle origin in the world frame, i.e. the translation part of
        `map -> base_link` (what `/odom` reports).
    lidar_offset : (3,) array_like
        The LiDAR's position in `base_link` (the `base_link -> lidar_link`
        translation).

    Returns
    -------
    (N, 3) ndarray
        Points in `lidar_link`, ready to be stamped as such.

    Derivation
    ----------
    With `map -> base_link = (R, t)` and `base_link -> lidar_link = (I, o)`,
    composition gives `map -> lidar_link = (R, t + R o)`. A world point `p`
    expressed in the LiDAR frame is therefore

        p_l = R^T (p - (t + R o)) = R^T (p - t) - o
    """
    points = np.asarray(points, dtype=np.float64)
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    lidar_offset = np.asarray(lidar_offset, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f'points must be (N, 3), got {points.shape}')
    # (p - t) @ R == R^T (p - t) for row vectors.
    return (points - translation) @ rotation - lidar_offset
