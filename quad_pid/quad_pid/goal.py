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