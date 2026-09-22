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