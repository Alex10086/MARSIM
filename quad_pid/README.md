# quad_pid

ROS 2 (Jazzy) PID flight controller for the MARSIM quadrotor simulation.
Replaces MARSIM's unstable `cascadePID`.

Outputs motor RPM on `/cmd_RPM`; consumes odometry (`/odom`), optional IMU
(`/imu`), and one of two command sources: position goals (`/cmd`) or velocity
commands (`/cmd_vel`).

## Control pipeline

Timer-driven at `control_rate` (default 100 Hz). Every tick runs four layers:

```
setpoint resolution (modes.py)          <- position goal | Twist | hold
  └─ S2 carrot: clamp pos_des to max_horiz_dist of the vehicle
       └─ L1  position PD -> world-frame desired acceleration
            └─ speed governor (max_horiz_speed), accel clamps, descend limit
                 └─ L2  accel -> roll/pitch (yaw-compensated) + total thrust
                      └─ L3  attitude error -> body torques
                           └─ L4  mixer -> 4 motor RPM
```

Pure logic lives in importable modules with no ROS dependency, so it is unit
tested directly: `modes.py` (command arbitration + setpoint integration),
`goal.py` (goal semantics), `geometry.py` (attitude math), `mixer.py`.

## Input modes

Selected by the `control_mode` parameter:

| `control_mode` | Source | Notes |
|---|---|---|
| `"position"` | `/cmd` (`PoseStamped`) | RViz 2D Goal Pose, or a planner's position setpoint |
| `"velocity"` | `/cmd_vel` (`Twist`) | Nav2 `controller_server`, teleop |
| `"auto"` *(default)* | whichever arrived last | Last-writer-wins between the two |

**`auto` semantics you must know about:** with `goal_latched: true` (the default,
which is what makes a one-shot RViz click stick), a latched position goal never
expires. So after a `Twist` stream stops, `auto` falls back to that goal and the
drone flies back to it. For pure Nav2/teleop operation use
`control_mode: "velocity"` (see `config/quad_pid_nav.yaml`), or `goal_latched:
false`.

Two useful debug parameters:

| Parameter | Default | Purpose |
|---|---|---|
| `publish_setpoint` | `false` | publish `/quad_pid/setpoint` (`PoseStamped`, world frame) — the setpoint the controller is actually chasing, after the S2 carrot clamp. In RViz this separates "the command was wrong" from "the controller failed to track it". |
| `verbose` | `false` (YAML sets `true`) | 1 Hz position/RPM/tilt log |

## Twist contract

Nav2 publishes `cmd_vel` in the robot **base** frame. `quad_pid` works in the
**world** frame, so the rotation happens in exactly one place
(`modes.body_to_world_velocity`, using the vehicle's *current* yaw). Getting this
wrong — or with the wrong yaw — produces a non-conservative force field and the
drone orbits instead of converging.

| Field | Unit | Meaning |
|---|---|---|
| `linear.x` | m/s | forward velocity in the **body** frame |
| `linear.y` | m/s | lateral velocity in the **body** frame |
| `linear.z` | m/s | ignored unless `twist_follow_z` is true |
| `angular.z` | rad/s | yaw **rate** — integrated into a yaw setpoint, *not* an angle |

Velocity commands are integrated into a world-frame position setpoint which is
**leashed** to the vehicle (`max_horiz_dist`, default 5 m) so it cannot wind up
while the drone is blocked. A velocity feedforward (`e_vel = vel_des - vel`)
removes the steady-state lag that pure position PD would have (KD/KP·v ≈ 2.8 m
at 1.4 m/s).

**A stale Twist freezes the setpoint and brakes; it never keeps integrating.**
Integration happens only inside the velocity branch of `SetpointResolver.update`,
so there is no code path that turns a stopped publisher into a runaway.

### QoS

`/cmd_vel` is subscribed with **BEST_EFFORT**, depth 1. Nav2's
`controller_server` publishes with `rclcpp::SystemDefaultsQoS()`
(**RELIABLE**). A BEST_EFFORT subscriber accepts a RELIABLE publisher, whereas a
RELIABLE subscriber would **silently receive nothing** from a BEST_EFFORT
(`SensorDataQoS`) publisher. Verify with
`ros2 topic info -v /cmd_vel_nav` — it must print `Subscription count: 1` and
both reliabilities.

## Parameters

All parameters are declared with defaults and re-read every tick, so
`ros2 param set` takes effect live — **except two startup-only parameters**:
`cmd_vel_topic` (read once to create the subscription) and `control_rate` (read
once to create the timer). Changing either needs a restart.

An unrecognised `control_mode` falls back to `auto`; the node logs a warning
once when it sees one, because that fallback silently enables the `auto`
gotcha described above.

Gains:

| Parameter | Default | Notes |
|---|---|---|
| `KP_XY`, `KD_XY` | 0.5, 1.0 | horizontal position PD (ωn ≈ 0.7–1.0 rad/s) |
| `KP_Z`, `KD_Z`, `KI_Z` | 1.0, 1.4, 0.0 | vertical (gravity is fed forward, so `KI_Z=0` suffices) |
| `KP_ATT`, `KD_ATT` | 0.2, 0.04 | roll/pitch (must match `I_xx = 2.64e-3`, ωn ≈ 8 rad/s) |
| `KP_YAW`, `KD_YAW` | 0.2, 0.05 | yaw |

Physical constants — **must match `mars_drone_sim`'s source**, a mismatch here
causes yaw over-control:

| Parameter | Default | Notes |
|---|---|---|
| `mass` | 1.9 | kg |
| `arm_length` | 0.22 | m |
| `k_F` | 2.694396e-8 | N/RPM² (= 3 × 8.98132e-9, X-config) |
| `k_T` | 3.508e-10 | N·m/RPM² |

Safety limits:

| Parameter | Default | Notes |
|---|---|---|
| `max_tilt_deg` | 20.0 | attitude clamp |
| `max_horiz_acc` | 3.0 | |
| `max_vert_acc_up` / `max_vert_acc_down` | 4.0 / -4.0 | must not clamp gravity (see git history) |
| `max_torque_xy` / `max_torque_z` | 1.0 / 0.3 | |
| `max_rpm` / `min_rpm` | 35000 / 0 | |
| `max_horiz_speed` | 2.0 | **hard ceiling on achieved** horizontal speed |
| `max_descend_speed` | 1.0 | |
| `max_horiz_dist` | 5.0 | carrot clamp *and* the Twist leash |
| `max_goal_dist` | 50.0 | reject position goals farther than this (near-horizon RViz clicks project kilometres away) |
| `goal_latched` | true | one-shot tools (RViz) never expire; set false for streaming planners |
| `goal_timeout` | 1.0 | staleness for non-latched goals |

Command sources and Twist:

| Parameter | Default | Notes |
|---|---|---|
| `control_mode` | `"auto"` | `position` \| `velocity` \| `auto` |
| `cmd_vel_topic` | `/cmd_vel` | **startup-only**; Nav2 profile uses `/cmd_vel_nav` |
| `cmd_vel_timeout` | 0.5 | s; older than this = stale = freeze |
| `twist_max_vx` / `twist_max_vy` | 1.5 / 1.5 | saturate the **command** |
| `twist_max_wz` | 0.8 | rad/s |
| `twist_max_vz` | 1.0 | m/s; rate limit for `linear.z` when `twist_follow_z` is on |
| `twist_target_height` | -1.0 | >0 = cruise at this altitude; ≤0 = latch current altitude on entering velocity mode |
| `twist_follow_z` | false | integrate `linear.z` (Nav2 always sends 0 — leave false) |
| `publish_setpoint` | false | |
| `use_imu` | true | Gyro damping is skipped when `/imu` is absent or stale |
| `control_rate` | 100.0 | Hz |

### `twist_max_*` vs `max_horiz_speed`

`twist_max_vx/vy` bound the **commanded** velocity, not the **achieved** one: the
position PD catching up to the advancing reference transiently peaks above it
(measured 1.9 m/s against a 1.5 limit, ~21% overshoot). `max_horiz_speed` is the
**achieved**-speed ceiling — the governor removes the outward acceleration
component as soon as speed reaches it, so the overshoot collapses to about one
tick's worth.

Setting `max_horiz_speed` equal to `twist_max_*` therefore makes the achieved
speed honour the command limit (this is what `config/quad_pid_nav.yaml` does, so
Nav2's `vx_max`/`vy_max` assumption holds). It does not make the cap
instantaneous — it bounds the overshoot to roughly a tick, not to zero. The cost
is diagonal throughput: a composite `√2 × 1.4 ≈ 1.98` command is capped at 1.4.
If you want full diagonal throughput, raise `max_horiz_speed` to `√2 × vx_max`
and accept the transient overshoot.

## Running

```bash
# MARSIM + quad_pid (default position profile)
ros2 launch quad_pid single_drone_quad_pid.launch.py

# Nav2 profile: control_mode=velocity, subscribing /cmd_vel_nav
ros2 launch quad_pid single_drone_quad_pid.launch.py \
  quad_pid_params:=$(ros2 pkg prefix quad_pid)/share/quad_pid/config/quad_pid_nav.yaml
```

### Testing Twist mode without Nav2

`twist_pub.py` drives `/cmd_vel` directly. Use it to decide whether a misbehaving
end-to-end run is the controller's fault or the planner's.

```bash
# straight line, then STOP publishing (the controller must brake and hold)
ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 duration:=5

# pre-rotate to -54 deg, then drive forward -- the orbit-bug regression
ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 yaw:=-54

# side slip at yaw=0
ros2 launch quad_pid twist_test.launch.py pattern:=strafe vy:=1.0

# square circuit
ros2 launch quad_pid twist_test.launch.py pattern:=square duration:=16

# or drive /cmd_vel by hand
ros2 launch quad_pid twist_test.launch.py run_twist_pub:=false
ros2 run quad_pid twist_pub.py --pattern forward --vx 1.0 --duration 5 --linger 6
```

Patterns: `zero`, `forward`, `backward`, `strafe`, `spin`, `square`. It publishes
at 20 Hz with Nav2's QoS, stops after `--duration`, then reports final position,
yaw and `drift_after_stop`.

### Hot-switching modes

```bash
ros2 param set /quad_pid_node control_mode position    # or velocity / auto
```

Mode switches are bumpless: entering velocity mode latches the setpoint reference
onto the vehicle's current pose, so the **command** does not jump. (`pos_des`
itself does change — from the old goal to the current pose — but that is what
latching means, and it is the reason the command stays smooth.)

## Gotchas learned the hard way

* **roll/pitch are body-frame.** The world-frame desired acceleration must be
  rotated by the current yaw first (`accel_to_attitude_yaw`). Without it, any
  non-zero yaw turns the controller into a non-conservative field and the drone
  orbits the origin instead of converging. Note that RViz's 2D Goal Pose writes
  the *drag direction* into the goal's yaw, so this is easy to hit by hand.
* **Do not divide thrust by `cos(tilt)`.** `a_des` already encodes both magnitude
  and direction; dividing double-counts thrust when tilted and causes runaway
  climb.
* **`max_vert_acc_up` must not clamp gravity.** The clamp applies to the PD part
  only, before gravity compensation is added back.
* **A 2D tool sends `z = 0`.** `sanitize_goal` treats `z <= 0` as "height
  unspecified" and holds the current altitude instead of diving to the ground.

## Tests

```bash
cd marsim_ws && PYTHONPATH=src/quad_pid python3 -m pytest src/quad_pid/test/ -q
```

`test_modes.py` covers the arbitration matrix, body→world rotation (including the
yaw ≠ 0 orbit regression), the leash, yaw wrap, and the stale-Twist freeze.
