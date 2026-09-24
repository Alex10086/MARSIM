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

# Integrator hygiene. A suspended process (debugger, scheduler stall) must not
# advance the setpoint by seconds' worth in one tick.
MAX_DT = 0.1
# Altitude band for `twist_follow_z`. A hard band is deliberate: this is a
# flight controller, and once integration is enabled `ref_z` has no other
# physical bound.
MIN_HEIGHT = 0.3
MAX_HEIGHT = 50.0


def clamp_dt(dt, max_dt=MAX_DT):
    """Clamp a measured control period into [0, max_dt]; NaN -> 0."""
    if not math.isfinite(dt):
        return 0.0
    return max(0.0, min(max_dt, dt))


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


def clamp_z_rate(vz, vmax):
    """Rate-limit `linear.z`, which clamp_twist's signature does not cover."""
    return clamp_twist(0.0, 0.0, vz, 0.0, 0.0, vmax)[2]


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

    An unrecognised `mode` deliberately falls back to 'auto' rather than
    raising: a typo in `control_mode` must not kill the flight loop. The node
    logs a one-time warning when it sees one, and the fallback is pinned by
    test_select_source_treats_an_unknown_mode_as_auto so it is intentional
    rather than accidental.
    """
    usable_pos = bool(goal_active)
    usable_vel = bool(twist_fresh)

    if mode == POSITION:
        return POSITION if usable_pos else HOLD
    if mode == VELOCITY:
        return VELOCITY if usable_vel else HOLD
    # 'auto' -- and any unrecognised value (see docstring).
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
        # True while HOLD should brake to `ref_xy` instead of holding the current
        # pose. Set by the velocity branch (so a stale stream decelerates to the
        # leashed reference) and cleared by the position branch. It is NOT
        # cleared by HOLD itself: braking takes many ticks, and it is NOT left
        # set by a position-mode excursion, because then forcing velocity mode
        # on with nothing to consume would fly the drone back to a reference
        # left over from an old session, arbitrarily far away.
        self.freeze = False
        # Last finite command, returned when odom goes non-finite (see update).
        self.last_good = None

    def update(self, *, mode, dt, cur_pos, cur_yaw,
               goal_active, goal_stamp, goal_xyz, goal_yaw,
               twist_fresh, twist_stamp,
               twist_vx, twist_vy, twist_vz, twist_wz, cfg):
        # Never consume a corrupt odom sample. Latching a non-finite value is
        # ABSORBING -- advance_setpoint's `d > leash` test is False for NaN, so
        # the NaN is rewritten into ref_xy every tick -- and the downstream
        # accel clamps turn NaN into a FULL-SCALE command (Python's min/max
        # keep the non-NaN operand, so max(-3, min(3, nan)) == 3), which means
        # the end-of-pipeline NaN guard never trips. Hold the last good command
        # and resume as soon as odom is sane again.
        if not (np.all(np.isfinite(cur_pos)) and math.isfinite(float(cur_yaw))):
            if self.last_good is None:
                return (np.zeros(3), 0.0, np.zeros(3), HOLD)
            return (self.last_good[0].copy(), self.last_good[1],
                    np.zeros(3), HOLD)

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
            vz_ff = 0.0
            if cfg['twist_follow_z']:
                # linear.z is the one field clamp_twist's signature does not
                # cover, so a FINITE but absurd value (1e308) would overflow
                # ref_z to inf within a few ticks and from there poison the S2
                # carrot (inf * 0.0 = NaN). Rate-limit it, then band the result.
                vz_ff = clamp_z_rate(twist_vz, cfg['twist_max_vz'])
                self.ref_z = min(max(self.ref_z + vz_ff * dt, MIN_HEIGHT),
                                 MAX_HEIGHT)
            self.freeze = True
            self.source = VELOCITY
            # z gets the same feedforward as xy. Without it, enabling
            # twist_follow_z would reintroduce the KD/KP*v tracking lag that the
            # xy feedforward exists to remove. When twist_follow_z is off this
            # is exactly 0.0, so the default profile is untouched.
            return self._out(np.array([self.ref_xy[0], self.ref_xy[1],
                                       self.ref_z]),
                             self.ref_yaw,
                             np.array([wx, wy, vz_ff]), VELOCITY)

        if src == POSITION:
            self.source = POSITION
            self.freeze = False
            return self._out(np.array(goal_xyz, dtype=float), float(goal_yaw),
                             np.zeros(3), POSITION)

        # HOLD. Braking to the reference left by a velocity session is a smooth
        # stop; holding the current pose is the pre-twist no-command behaviour
        # and the right answer for every other route into HOLD (a goal that
        # expired, control_mode forcing a source with nothing to consume).
        self.source = HOLD
        if not self.freeze or self.ref_xy is None:
            return self._out(np.array(cur_pos, dtype=float), float(cur_yaw),
                             np.zeros(3), HOLD)
        return self._out(np.array([self.ref_xy[0], self.ref_xy[1], self.ref_z]),
                         self.ref_yaw, np.zeros(3), HOLD)

    def _out(self, pos_des, yaw_des, vel_des, src):
        """Cache the last finite command, then return the update tuple."""
        pos_des = np.asarray(pos_des, dtype=float)
        if np.all(np.isfinite(pos_des)) and math.isfinite(float(yaw_des)):
            self.last_good = (pos_des.copy(), float(yaw_des))
        return (pos_des, float(yaw_des), vel_des, src)

    def _latch(self, cur_pos, cur_yaw, cfg):
        """Bumpless transfer: put the reference on the vehicle's current state."""
        self.ref_xy = (float(cur_pos[0]), float(cur_pos[1]))
        self.ref_yaw = float(cur_yaw)
        th = cfg.get('twist_target_height', -1.0)
        self.ref_z = float(th) if th > 0.0 else float(cur_pos[2])
