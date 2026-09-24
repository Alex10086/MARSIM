"""Control-source arbitration and Twist -> setpoint integration.

Pure logic: this module must NOT import rclpy. Keeping every decision here
(rather than in the node's timer callback) is what makes the whole mode
machine — arbitration, integration, bumpless transfer — unit-testable without
a running ROS graph, following the goal.py / geometry.py precedent.

Frame contract
--------------
Nav2 publishes cmd_vel in the robot *base* frame (body). Every consumer inside
quad_pid works in the *world* frame, so `body_to_world_velocity` is the single
place where that rotation happens. Doing it anywhere else — or with the wrong
yaw — reproduces the orbit/limit-cycle bug fixed in commit fa1f0e2.
"""

import math

POSITION = 'position'
VELOCITY = 'velocity'
HOLD = 'hold'


def clamp_twist(vx, vy, wz, vmax_x, vmax_y, wmax):
    """Saturate a body-frame velocity command per axis, sanitising NaN/Inf.

    ±inf is a legible "as fast as possible" command -> saturate to the limit.
    NaN becomes 0.0, so a malformed command can never reach the integrator.
    """
    def _sat_inf(v, lim):
        if math.isnan(v):
            return 0.0
        if v == math.inf:
            return lim
        if v == -math.inf:
            return -lim
        return max(-lim, min(lim, v))

    return (_sat_inf(vx, vmax_x), _sat_inf(vy, vmax_y), _sat_inf(wz, wmax))


def body_to_world_velocity(vx, vy, yaw):
    """Rotate a body-frame horizontal velocity into the world frame: R_z(yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx - s * vy, s * vx + c * vy)


def advance_yaw(ref_yaw, wz, dt):
    """Integrate a yaw *rate* into a yaw setpoint, wrapped to (-π, π]."""
    return math.atan2(math.sin(ref_yaw + wz * dt), math.cos(ref_yaw + wz * dt))


def advance_setpoint(ref_xy, v_world, dt, leash, cur_xy):
    """Advance the integrated setpoint by v·dt, then leash it to the vehicle.

    The leash is what replaces an external bridge node's `lookahead`: it keeps
    the reference within `leash` metres of the ACTUAL position, so the setpoint
    cannot wind up while the vehicle is blocked, and the position PD never sees
    an error larger than the existing max_horiz_dist carrot.
    """
    nx = ref_xy[0] + v_world[0] * dt
    ny = ref_xy[1] + v_world[1] * dt
    dx, dy = nx - cur_xy[0], ny - cur_xy[1]
    d = math.hypot(dx, dy)
    if d > leash and d > 1e-9:
        k = leash / d
        nx = cur_xy[0] + dx * k
        ny = cur_xy[1] + dy * k
    return (nx, ny)


def select_source(mode, goal_active, goal_stamp, twist_fresh, twist_stamp):
    """Arbitrate between the position and velocity command sources.

    'position' / 'velocity' force a source (subject to that source being
    usable); 'auto' is last-writer-wins between the two. Staleness is folded
    into `goal_active` / `twist_fresh` by the caller.
    """
    usable_pos = bool(goal_active)
    usable_vel = bool(twist_fresh)

    if mode == POSITION:
        return POSITION if usable_pos else HOLD
    if mode == VELOCITY:
        return VELOCITY if usable_vel else HOLD
    # auto
    if usable_pos and usable_vel:
        return POSITION if goal_stamp >= twist_stamp else VELOCITY
    if usable_pos:
        return POSITION
    if usable_vel:
        return VELOCITY
    return HOLD
