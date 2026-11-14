"""Validate and allocate the example mission to UAV and UGV."""
import argparse
import json
from pathlib import Path
from task_allocator import allocate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", default="config/inspection_mission.json")
    parser.add_argument("--output", default="config/assignments.json")
    args = parser.parse_args()
    mission = json.loads(Path(args.mission).read_text(encoding="utf-8"))
    if mission.get("world_frame") != "EUN":
        raise SystemExit("This starter expects the Webots world to use EUN (Y-up).")
    robot_caps = mission["robots"]
    assignments = allocate(mission["tasks"], robot_caps)
    output = {
        "world_frame": mission["world_frame"],
        "coordinate_note": mission.get("coordinate_note", ""),
        "assignments": assignments,
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    for robot, tasks in assignments.items():
        print(f"{robot}: {', '.join(t['id'] for t in tasks) or '(none)'}")
    print(f"Wrote {out_path}")

if __name__ == "__main__":
    main()
