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

import numpy as np

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


class SetpointResolver:
    """Resolve (pos_des, yaw_des, vel_des) for one control tick.

    Holds only the latched velocity-mode reference. Pure Python + numpy: no ROS
    imports, so the entire mode machine (arbitration, integration, bumpless
    transfer, hold semantics) is unit-testable outside a running graph, and the
    node stays a thin orchestrator around it.
    """

    def __init__(self, leash):
        self.leash = float(leash)
        self.source = None          # last active source, for bumpless transfer
        self.ref_xy = None          # None until velocity mode is first entered
        self.ref_z = 0.0
        self.ref_yaw = 0.0

    def update(self, *, mode, dt, cur_pos, cur_yaw,
               goal_active, goal_stamp, goal_xyz, goal_yaw,
               twist_fresh, twist_stamp,
               twist_vx, twist_vy, twist_vz, twist_wz, cfg):
        src = select_source(mode, goal_active, goal_stamp, twist_fresh, twist_stamp)

        if src == VELOCITY:
            if self.source != VELOCITY:
                self._latch(cur_pos, cur_yaw, cfg)      # bumpless transfer
            vx, vy, wz = clamp_twist(twist_vx, twist_vy, twist_wz,
                                     cfg['twist_max_vx'], cfg['twist_max_vy'],
                                     cfg['twist_max_wz'])
            wx, wy = body_to_world_velocity(vx, vy, cur_yaw)
            self.ref_xy = advance_setpoint(self.ref_xy, (wx, wy), dt,
                                           self.leash, (cur_pos[0], cur_pos[1]))
            self.ref_yaw = advance_yaw(self.ref_yaw, wz, dt)
            if cfg['twist_follow_z'] and math.isfinite(twist_vz):
                self.ref_z += twist_vz * dt
            self.source = VELOCITY
            return (np.array([self.ref_xy[0], self.ref_xy[1], self.ref_z]),
                    self.ref_yaw,
                    np.array([wx, wy, 0.0]), VELOCITY)

        if src == POSITION:
            self.source = POSITION
            return (np.array(goal_xyz, dtype=float), float(goal_yaw),
                    np.zeros(3), POSITION)

        # HOLD
        self.source = HOLD
        if self.ref_xy is None:
            # Never been in velocity mode -> exactly the pre-twist behaviour
            return (np.array(cur_pos, dtype=float), float(cur_yaw),
                    np.zeros(3), HOLD)
        # A velocity session ended -> freeze the (leashed) reference. The leash
        # guarantees it is within `leash` metres, so the drone decelerates to a
        # stop just ahead instead of drifting on.
        return (np.array([self.ref_xy[0], self.ref_xy[1], self.ref_z]),
                self.ref_yaw, np.zeros(3), HOLD)

    def _latch(self, cur_pos, cur_yaw, cfg):
        """Bumpless transfer: put the reference on the vehicle's current state."""
        self.ref_xy = (float(cur_pos[0]), float(cur_pos[1]))
        self.ref_yaw = float(cur_yaw)
        th = cfg.get('twist_target_height', -1.0)
        self.ref_z = float(th) if th > 0.0 else float(cur_pos[2])
