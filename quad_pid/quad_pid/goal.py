"""Goal setpoint activity logic.

Two goal-delivery semantics must both work:

* **latched / one-shot** — RViz's 2D Goal Pose publishes the goal once per
  click and then goes silent. The goal must remain the active setpoint
  indefinitely (until a new one arrives). A position-hold controller tracking
  a latched setpoint is inherently safe; the real safety net is *odom* going
  stale, not the goal going stale.

* **streaming** — a planner republishes the setpoint every cycle. If the
  stream stops for longer than `goal_timeout`, the goal is considered lost
  and the controller holds the current position.
"""


def goal_is_active(has_goal, goal_age, goal_timeout, latched):
    """Return True if the stored goal should be used as the setpoint.

    Parameters
    ----------
    has_goal : bool
        Whether any goal has ever been received.
    goal_age : float
        Seconds since the goal was received.
    goal_timeout : float
        Age above which a *streaming* goal is considered stale.
    latched : bool
        True for one-shot/latched goals (never expire); False for streaming
        goals (expire after `goal_timeout`).
    """
    if not has_goal:
        return False
    if latched:
        return True
    return goal_age <= goal_timeout


def sanitize_goal(gx, gy, gz, cx, cy, cz, max_goal_dist):
    """Sanitize a raw goal against 2D-tool / out-of-range artefacts.

    Returns ``(x, y, z, ok)``.

    * ``z <= 0`` is treated as "height unspecified": this is exactly what a 2D
      tool such as RViz's 2D Goal Pose emits (it has no z and always publishes
      0), so the current height is kept rather than diving to the ground.
    * If the goal's horizontal distance from the vehicle (cx, cy) exceeds
      ``max_goal_dist`` it is rejected (``ok=False``).  A click near the
      horizon projects onto the ground plane kilometres away.
    * NaN / inf coordinates are rejected.
    """
    import math as _math
    x, y, z = float(gx), float(gy), float(gz)
    if not _math.isfinite(x) or not _math.isfinite(y) or not _math.isfinite(z):
        return x, y, z, False
    if z <= 0.0:
        z = float(cz)
    if _math.hypot(x - cx, y - cy) > max_goal_dist:
        return x, y, z, False
    return x, y, z, True
