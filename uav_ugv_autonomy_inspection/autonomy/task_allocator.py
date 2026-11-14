"""Deterministic capability-based task allocation (no ROS dependency)."""
from __future__ import annotations


def allocate(tasks: list[dict], robots: dict[str, dict]) -> dict[str, list[dict]]:
    """Assign each task to a capable robot; priority breaks ties.

    Tasks requiring capabilities no robot has are returned under '_unassigned'.
    The score is explainable: exact capability coverage first, then fewer excess
    capabilities (specialized robot), then stable robot-name tie-break.
    """
    result = {name: [] for name in robots}
    result["_unassigned"] = []
    for task in sorted(tasks, key=lambda t: (-int(t.get("priority", 0)), t["id"])):
        required = set(task.get("required", []))
        candidates = []
        for robot_name, robot in robots.items():
            caps = set(robot.get("capabilities", []))
            if required.issubset(caps):
                candidates.append((len(caps - required), robot_name))
        if not candidates:
            result["_unassigned"].append(task)
            continue
        _, chosen = sorted(candidates)[0]
        result[chosen].append(task)
    return result
