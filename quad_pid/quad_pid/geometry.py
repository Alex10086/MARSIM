import math


def accel_to_attitude(a_x: float, a_y: float, a_z: float) -> tuple[float, float]:
    """Convert desired acceleration (world frame) to desired roll and pitch.

    The desired thrust direction is normalize([a_x, a_y, a_z + g]).
    a_z already includes gravity compensation (i.e., a_z ≈ g for hover).
    Returns (roll_des, pitch_des) in radians.
    """
    # Desired body-z axis in world frame = direction of acceleration
    z_x = a_x
    z_y = a_y
    z_z = a_z

    # pitch: rotation about body y-axis
    pitch_des = math.atan2(z_x, z_z)

    # roll: rotation about body x-axis
    roll_des = math.atan2(-z_y, math.sqrt(z_x * z_x + z_z * z_z))

    return roll_des, pitch_des


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about world Z) from quaternion.

    Uses the standard ZYX Euler angle extraction:
      yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy² + qz²))
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle(target: float, current: float) -> float:
    """Shortest signed angle from current to target, result in [-π, π]."""
    diff = target - current
    # Wrap to [-π, π]
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return diff

def limit_horizontal_speed(a_x: float, a_y: float,
                           v_x: float, v_y: float,
                           max_speed: float) -> tuple[float, float]:
    """Cap horizontal speed by removing only the outward acceleration.

    The position PD controller has no explicit speed setpoint: speed emerges as
    (KP_XY/KD_XY)*distance. When the vehicle is already moving at `max_speed`,
    any commanded acceleration with a positive component along the velocity
    direction would increase speed further; that outward component is removed.
    Braking (backward) and perpendicular components are preserved, so the drone
    can still slow down and turn. Returns the adjusted (a_x, a_y).
    """
    speed = math.hypot(v_x, v_y)
    if speed < max_speed or speed < 1e-9:
        return a_x, a_y
    ux, uy = v_x / speed, v_y / speed          # unit velocity
    outward = a_x * ux + a_y * uy              # accel component along velocity
    if outward <= 0.0:
        return a_x, a_y                        # braking / neutral: keep
    return a_x - outward * ux, a_y - outward * uy


def accel_to_attitude_yaw(a_x: float, a_y: float, a_z: float,
                          yaw: float) -> tuple[float, float]:
    """Like `accel_to_attitude` but compensates for the vehicle's yaw.

    The desired thrust direction is the world-frame unit vector of
    ``[a_x, a_y, a_z]`` (a_z already includes gravity compensation).  Because
    roll and pitch are *body*-frame quantities, this direction must first be
    rotated from the world frame into the body frame using yaw:

        [b_x, b_y, b_z] = R_z(-yaw) @ [a_x, a_y, a_z]

    Omitting this rotation means the realized horizontal acceleration is the
    commanded one rotated by -yaw, which is a non-conservative force field: the
    drone then traces a stable limit cycle (orbits) instead of converging.

    Returns (roll_des, pitch_des) in radians. At yaw == 0 this reduces exactly
    to `accel_to_attitude`.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    b_x = c * a_x + s * a_y
    b_y = -s * a_x + c * a_y
    b_z = a_z

    pitch_des = math.atan2(b_x, b_z)
    roll_des = math.atan2(-b_y, math.sqrt(b_x * b_x + b_z * b_z))
    return roll_des, pitch_des
