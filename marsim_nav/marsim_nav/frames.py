"""Frame names shared across the MARSIM/Nav2 integration.

MARSIM emits everything in a 'world' frame (ENU) and publishes NO TF at all --
`quadrotor_dynamics_node` only publishes the /odom message. Nav2 requires a
TF tree, so this package supplies it.

In simulation map == odom == world (perfect odometry, no drift), so they are
joined by static identity transforms. That is also why AMCL is deliberately
absent: there is nothing to localise against.
"""

MAP = "map"
ODOM = "odom"
WORLD = "world"
BASE = "base_link"
LIDAR = "lidar_link"

# Header frame ids actually written by the MARSIM C++ nodes. Do not change
# these here -- they come from source; we adapt the TF tree to them instead.
MARSIM_ODOM_FRAME = "world"
MARSIM_CLOUD_FRAME = "world"
MARSIM_IMU_FRAME = "/quadrotor"
