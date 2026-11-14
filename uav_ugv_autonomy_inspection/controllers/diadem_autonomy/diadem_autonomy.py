"""RigBetel Labs Diadem UGV autonomy controller with differential/skid-steer navigation,
YDLidar obstacle avoidance, and bidirectional communication for mutual rendezvous with Crazyflie UAV.

Coordinate system: EUN (X=east, Y=up, Z=north).
Ground motion operates on the X-Z plane.

Diadem Robot Architecture:
    Wheel Radius: 0.18 m
    Track Width:  0.60 m
    Wheel Base:   0.56 m
    Drive Mode:   4-wheel skid-steer
        FL_wheel_joint    FR_wheel_joint
        RL_wheel_joint    RR_wheel_joint
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

# ── Webots controller import ──────────────────────────────────────────
try:
    from controller import Supervisor, Robot
except ImportError:
    from controller import Robot  # type: ignore
    Supervisor = Robot

# ── Add shared utilities to path ──────────────────────────────────────
_CTRL_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_CTRL_DIR / "shared"))

from coord_utils import (
    altitude,
    angle_diff,
    clamp,
    horizontal_distance,
    horizontal_position,
    rendezvous_point_below,
    world_position,
    heading_to_target,
    ground_projection,
    yaw_from_imu,
)
from comm_protocol import (
    CommMessage,
    MailboxComm,
    ID_UAV,
    MSG_ASSISTANCE_REQUEST,
    MSG_REQUEST_ACK,
    MSG_RENDEZVOUS_TARGET,
    MSG_RENDEZVOUS_ACCEPTED,
    MSG_ARRIVED,
    MSG_INSPECTION_REPORT,
    MSG_ASSISTANCE_COMPLETE,
    MSG_CANCEL_REQUEST,
    MSG_HEARTBEAT,
    MSG_ERROR,
    make_request_id,
)

ID_DIADEM = "diadem"

# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════
_CFG_PATH = _CTRL_DIR.parent / "config" / "rendezvous_config.json"
try:
    _CFG = json.loads(_CFG_PATH.read_text(encoding="utf-8"))
except Exception:
    _CFG = {}

_DIADEM_CFG   = _CFG.get("diadem", _CFG.get("ugv", {}))
_COMM_CFG     = _CFG.get("communication", {})
_TEST_CFG     = _CFG.get("test_scenarios", {})
_NAV_CFG      = _DIADEM_CFG.get("navigation", {})
_INSP_TARGETS = _CFG.get("inspection_targets", {})
_CAPABILITIES = _CFG.get("capabilities", {})
_VALVE_TARGET = _INSP_TARGETS.get("high_valve_1", {})

# Diadem Drive parameters
MAX_SPEED     = _DIADEM_CFG.get("max_speed_mps", 0.50)
MAX_WHEEL_VEL = _DIADEM_CFG.get("max_wheel_speed_radps", 10.0)
WHEEL_RADIUS  = _DIADEM_CFG.get("wheel_radius_m", 0.18)
TRACK_WIDTH   = _DIADEM_CFG.get("track_width_m", 0.60)
ARRIVE_TOL    = _DIADEM_CFG.get("arrival_tolerance_m", 0.45)
APPROACH_SLOW = _DIADEM_CFG.get("approach_slowdown_distance_m", 1.5)
OBS_STOP_DIST = _DIADEM_CFG.get("obstacle_stop_distance_m", 0.50)
OBS_SLOW_DIST = _DIADEM_CFG.get("obstacle_slowdown_distance_m", 1.5)
TARGET_DEADBAND = _DIADEM_CFG.get("target_update_deadband_m", 0.5)

# Navigation gains
KP_LIN   = _NAV_CFG.get("kp_linear", 1.0)
KP_ANG   = _NAV_CFG.get("kp_angular", 3.0)
HEAD_TOL = _NAV_CFG.get("heading_tolerance_rad", 0.1)
OBS_GAIN = _NAV_CFG.get("obstacle_avoidance_gain", 1.5)

# Communication
COMM_TTL     = _COMM_CFG.get("message_ttl_s", 60.0)
COMM_TIMEOUT = _COMM_CFG.get("request_timeout_s", 30.0)
HB_INTERVAL  = _COMM_CFG.get("heartbeat_interval_s", 5.0)

# Test
DEMO_MODE       = _TEST_CFG.get("demo_mode", "ugv_calls_uav_high_valve")
DIADEM_CALL_DELAY = _TEST_CFG.get("ugv_calls_uav_delay_s", 5.0)


# ═══════════════════════════════════════════════════════════════════════
# UGV State Machine
# ═══════════════════════════════════════════════════════════════════════
class DiademState:
    IDLE                 = "IDLE"
    PATROL_TO_STATION    = "PATROL_TO_STATION"
    CAPABILITY_EVAL      = "CAPABILITY_EVAL"
    REQUEST_GENERATION   = "REQUEST_GENERATION"
    REQUEST_TRANSMISSION = "REQUEST_TRANSMISSION"
    WAITING_FOR_UAV      = "WAITING_FOR_UAV"
    TARGET_VALIDATION    = "TARGET_VALIDATION"
    PATH_PLANNING        = "PATH_PLANNING"
    NAVIGATING           = "NAVIGATING"
    RENDEZVOUS_APPROACH  = "RENDEZVOUS_APPROACH"
    STOPPED_AT_TARGET    = "STOPPED_AT_TARGET"
    ARRIVAL_CONFIRMATION = "ARRIVAL_CONFIRMATION"
    ASSISTANCE_COMPLETE  = "ASSISTANCE_COMPLETE"
    ERROR_RECOVERY       = "ERROR_RECOVERY"


# ═══════════════════════════════════════════════════════════════════════
# Obstacle detector from LiDAR
# ═══════════════════════════════════════════════════════════════════════
class LidarObstacleDetector:
    """12-sector obstacle detection from 360-degree LiDAR."""
    NUM_SECTORS = 12

    def __init__(self):
        self.sector_min = [float("inf")] * self.NUM_SECTORS

    def update(self, lidar_device) -> None:
        try:
            range_image = lidar_device.getRangeImage()
        except Exception:
            return
        if not range_image:
            return
        n_points = len(range_image)
        self.sector_min = [float("inf")] * self.NUM_SECTORS
        for i, r in enumerate(range_image):
            if r <= 0 or math.isinf(r) or math.isnan(r):
                continue
            angle = 2.0 * math.pi * i / n_points
            sector = int(angle / (2.0 * math.pi) * self.NUM_SECTORS) % self.NUM_SECTORS
            if r < self.sector_min[sector]:
                self.sector_min[sector] = r

    def front_distance(self) -> float:
        return self.sector_min[0]

    def front_left_distance(self) -> float:
        return self.sector_min[1]

    def front_right_distance(self) -> float:
        return self.sector_min[self.NUM_SECTORS - 1]

    def is_path_blocked(self, threshold: float = 0.50) -> bool:
        return self.front_distance() < threshold

    def compute_avoidance_angular(self) -> float:
        fl_d = self.front_left_distance()
        fr_d = self.front_right_distance()
        # Avoid only if obstacles are close in the forward sector
        if fl_d < 0.65 or fr_d < 0.65:
            # Steer proportionally away from the closer side
            diff = fr_d - fl_d
            return clamp(diff * 1.5, -0.8, 0.8)
        return 0.0


# ═══════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════
def main():
    try:
        robot = Supervisor()
        print("[INIT] Diadem running with Supervisor API")
    except Exception as exc:
        print(f"[WARN] Supervisor instantiation failed ({exc}), falling back to Robot")
        robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    print("=" * 60)
    print("  RIGBETEL LABS DIADEM UGV AUTONOMY CONTROLLER")
    print(f"  Timestep: {timestep} ms | Wheel Radius: {WHEEL_RADIUS} m | Track: {TRACK_WIDTH} m")
    print("=" * 60)

    # ── Motors initialisation ──────────────────────────────────────────
    motor_aliases = {
        "FL": ["FL_wheel_joint", "fl_wheel_joint", "front_left_wheel_joint"],
        "FR": ["FR_wheel_joint", "fr_wheel_joint", "front_right_wheel_joint"],
        "RL": ["RL_wheel_joint", "rl_wheel_joint", "rear_left_wheel_joint"],
        "RR": ["RR_wheel_joint", "rr_wheel_joint", "rear_right_wheel_joint"],
    }
    motors = {}
    for key, aliases in motor_aliases.items():
        motor = None
        for name in aliases:
            m = robot.getDevice(name)
            if m is not None:
                motor = m
                break
        if motor is None:
            print(f"[FATAL] Motor for {key} not found among {aliases}!")
            return
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors[key] = motor
    print(f"[INIT] Diadem motors mapped: {list(motors.keys())}")

    # ── Sensor initialisation ─────────────────────────────────────────
    lidar = robot.getDevice("laser") or robot.getDevice("lidar")
    if lidar:
        lidar.enable(timestep)
        try:
            lidar.enablePointCloud()
        except Exception:
            pass
        print("[INIT] YDLidar enabled")
    else:
        print("[WARN] LiDAR device not found!")

    imu_compass = robot.getDevice("imu compass") or robot.getDevice("compass")
    imu_gyro = robot.getDevice("imu gyro") or robot.getDevice("gyro")
    imu_accel = robot.getDevice("imu accelerometer") or robot.getDevice("accelerometer")
    imu_inertial = robot.getDevice("imu inertial_unit") or robot.getDevice("inertial_unit")

    for dev in (imu_compass, imu_gyro, imu_accel, imu_inertial):
        if dev:
            dev.enable(timestep)

    gps = robot.getDevice("gps")
    if gps:
        gps.enable(timestep)
        print("[INIT] GPS enabled")

    # ── Communication ─────────────────────────────────────────────────
    mailbox_dir = _CTRL_DIR / "comm_mailbox"
    comm = MailboxComm(ID_DIADEM, mailbox_dir, peer_id=ID_UAV)
    comm.clear_inbox()
    print(f"[INIT] Communication: mailbox at {mailbox_dir} (Peer: {ID_UAV})")

    obs_detector = LidarObstacleDetector()

    # ── State machine ─────────────────────────────────────────────────
    state = DiademState.IDLE
    target_pos = None
    current_request_id = None
    request_time = 0.0
    request_counter = 0
    last_hb_time = 0.0
    uav_last_known_pos = None
    diadem_initiated_request = False
    idle_timer = 0.0
    arrival_report_sent = False

    for _ in range(5):
        robot.step(timestep)

    initial_pos = _get_position(robot, gps, imu_inertial)
    print(f"[INIT] Diadem initial position: {initial_pos}")
    print(f"[STATE] → {state}")

    # ══════════════════════════════════════════════════════════════════
    # Main loop
    # ══════════════════════════════════════════════════════════════════
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()

        pos = _get_position(robot, gps, imu_inertial)
        heading = _get_heading(robot, imu_compass, imu_inertial)

        if lidar:
            obs_detector.update(lidar)

        # ── Receive messages ──────────────────────────────────────────
        messages = comm.receive(sim_time, ttl=COMM_TTL)
        for msg in messages:
            print(f"[RECV] {msg.message_type} | id={msg.request_id} | from={msg.sender}")

            if msg.message_type == MSG_ASSISTANCE_REQUEST and msg.sender == ID_UAV:
                if state == DiademState.IDLE:
                    current_request_id = msg.request_id
                    uav_last_known_pos = msg.position
                    request_time = sim_time
                    state = DiademState.TARGET_VALIDATION
                    print(f"[STATE] → {state} (request from UAV: {current_request_id})")
                    comm.send_ack(current_request_id, sim_time, position=list(pos))

            elif msg.message_type == MSG_REQUEST_ACK and current_request_id and msg.request_id == current_request_id:
                if state == DiademState.WAITING_FOR_UAV:
                    uav_last_known_pos = msg.position
                    print(f"[INFO] Crazyflie UAV acknowledged request {current_request_id}")

            elif msg.message_type == MSG_RENDEZVOUS_TARGET and current_request_id and msg.request_id == current_request_id:
                print(f"[INFO] Received rendezvous target: {msg.target}")

            elif msg.message_type == MSG_ARRIVED and msg.sender == ID_UAV:
                if current_request_id and msg.request_id == current_request_id:
                    uav_last_known_pos = msg.position
                    print(f"[INFO] Crazyflie UAV arrived on station! pos={msg.position}")

            elif msg.message_type == MSG_INSPECTION_REPORT and msg.sender == ID_UAV:
                if current_request_id and msg.request_id == current_request_id:
                    uav_last_known_pos = msg.position
                    findings = msg.findings or {}
                    target_name = msg.target_name or "Elevated_Valve_1"
                    print("\n" + "=" * 60)
                    print("  [DIADEM UGV] AERIAL INSPECTION REPORT RECEIVED FROM CRAZYFLIE UAV")
                    print("=" * 60)
                    print(f"  Target:           {target_name}")
                    print(f"  Status:           {findings.get('valve_status', 'OPERATIONAL')}")
                    print(f"  Visual Findings:  {findings.get('visual_inspection', 'PASS')}")
                    print(f"  Surface Temp:     {findings.get('surface_temp_c', 42.5)} °C")
                    print(f"  Confidence:       {findings.get('confidence', 0.98) * 100:.1f}%")
                    print("=" * 60 + "\n")

                    summary_path = mailbox_dir / "inspection_summary.json"
                    try:
                        summary_data = {
                            "request_id": msg.request_id,
                            "timestamp": sim_time,
                            "ugv_platform": "RigBetel Labs Diadem",
                            "target_name": target_name,
                            "findings": findings,
                            "uav_position": msg.position,
                            "diadem_position": list(pos),
                        }
                        summary_path.write_text(json.dumps(summary_data, indent=2), encoding="utf-8")
                        print(f"[LOG] Saved inspection summary to {summary_path}")
                    except Exception as exc:
                        print(f"[WARN] Failed to write inspection summary: {exc}")

                    comm.send_complete(current_request_id, sim_time)
                    state = DiademState.ASSISTANCE_COMPLETE
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_ASSISTANCE_COMPLETE:
                if current_request_id and msg.request_id == current_request_id:
                    state = DiademState.ASSISTANCE_COMPLETE
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_CANCEL_REQUEST:
                if current_request_id and msg.request_id == current_request_id:
                    _stop_wheels(motors)
                    state = DiademState.IDLE
                    current_request_id = None
                    target_pos = None

            elif msg.message_type == MSG_HEARTBEAT and msg.sender == ID_UAV:
                uav_last_known_pos = msg.position

        # ── Heartbeat ─────────────────────────────────────────────────
        if sim_time - last_hb_time >= HB_INTERVAL:
            comm.send_heartbeat(sim_time, list(pos))
            last_hb_time = sim_time

        # ── State Machine ─────────────────────────────────────────────
        if state == DiademState.IDLE:
            _stop_wheels(motors)
            idle_timer += dt

            if not diadem_initiated_request and idle_timer >= 2.0:
                station_coord = _VALVE_TARGET.get("ugv_station_position", [0.0, 0.18, 8.0])
                target_pos = [float(station_coord[0]), float(station_coord[1]), float(station_coord[2])]
                state = DiademState.PATROL_TO_STATION
                print("\n" + "=" * 60)
                print("  DIADEM UGV: INDUSTRIAL CORRIDOR PATROL")
                print(f"  Target Inspection Station: {target_pos}")
                print(f"  Chassis: 4WD Skid-Steer (RigBetel Labs Diadem)")
                print("=" * 60)
                print(f"[STATE] → {state}")

        elif state == DiademState.PATROL_TO_STATION:
            if target_pos is None:
                state = DiademState.ERROR_RECOVERY
                continue

            h_dist = horizontal_distance(pos, target_pos)
            if h_dist < 0.65:
                _stop_wheels(motors)
                print(f"\n[NAV] Diadem arrived at High Valve Station! (dist={h_dist:.2f} m, pos={[round(p,2) for p in pos]})")
                state = DiademState.CAPABILITY_EVAL
                print(f"[STATE] → {state}\n")
                continue

            # Target heading with cross-track centering (keeps Diadem centered along aisle X=0)
            dx = (target_pos[0] - pos[0]) + (-pos[0] * 1.5)
            dz = target_pos[2] - pos[2]
            desired_heading = math.atan2(dx, dz)
            heading_err = angle_diff(desired_heading, heading)

            front_clear = not obs_detector.is_path_blocked(0.40)
            avoidance_steer = obs_detector.compute_avoidance_angular()

            if not front_clear:
                linear = 0.0
                angular = -1.0 if obs_detector.front_left_distance() < obs_detector.front_right_distance() else 1.0
            else:
                if abs(heading_err) > 0.40:
                    linear = 0.0
                    angular = clamp(1.2 * heading_err, -1.0, 1.0)
                else:
                    angular = clamp(1.2 * heading_err + avoidance_steer, -1.0, 1.0)
                    alignment = max(0.0, 1.0 - abs(heading_err) / math.pi)
                    dist_factor = min(1.0, max(0.25, h_dist / 1.5))
                    linear = 0.40 * alignment * dist_factor

            # Periodic telemetry log
            if int(sim_time * 10) % 25 == 0:
                print(f"[DIADEM DRIVE] Pos={[round(p,2) for p in pos]} | Target={target_pos} | Dist={h_dist:.2f}m | HeadErr={math.degrees(heading_err):.1f}° | v={linear:.2f}m/s, w={angular:.2f}rad/s")

            _set_drive(motors, linear, angular)

        elif state == DiademState.CAPABILITY_EVAL:
            _stop_wheels(motors)
            valve_elevation = _VALVE_TARGET.get("elevation_m", 1.8)
            diadem_max_reach = 0.85
            deficit = valve_elevation - diadem_max_reach

            print("\n" + "=" * 60)
            print("  [DIADEM UGV] CAPABILITY EVALUATION")
            print(f"  Target:            {_VALVE_TARGET.get('name', 'Elevated_Valve_1')}")
            print(f"  Valve Elevation:   {valve_elevation:.2f} m")
            print(f"  Diadem Mast Reach: {diadem_max_reach:.2f} m")
            print(f"  Deficit:           +{deficit:.2f} m (Out of ground reach)")
            print("  Action: Requesting Crazyflie UAV for Aerial Inspection Rendezvous")
            print("=" * 60 + "\n")

            state = DiademState.REQUEST_GENERATION
            print(f"[STATE] → {state}")

        elif state == DiademState.REQUEST_GENERATION:
            request_counter += 1
            current_request_id = make_request_id("diadem_insp", request_counter)
            state = DiademState.REQUEST_TRANSMISSION
            print(f"[STATE] → {state} (id={current_request_id})")

        elif state == DiademState.REQUEST_TRANSMISSION:
            comm.send_assistance_request(
                current_request_id,
                sim_time,
                position=list(pos),
                assistance_type="elevated_inspection",
                target=_VALVE_TARGET.get("valve_position", [-1.0, 1.8, 8.0]),
                inspection_position=_VALVE_TARGET.get("uav_inspection_position", [-0.5, 1.8, 8.0]),
                target_name=_VALVE_TARGET.get("name", "Elevated_Valve_1"),
            )
            print(f"[SENT] ASSISTANCE_REQUEST from Diadem to Crazyflie UAV | id={current_request_id}")
            request_time = sim_time
            diadem_initiated_request = True
            state = DiademState.WAITING_FOR_UAV
            print(f"[STATE] → {state}")

        elif state == DiademState.WAITING_FOR_UAV:
            _stop_wheels(motors)
            if sim_time - request_time > COMM_TIMEOUT:
                print(f"[TIMEOUT] No UAV response after {COMM_TIMEOUT}s")
                state = DiademState.ERROR_RECOVERY
                print(f"[STATE] → {state}")

        elif state == DiademState.ASSISTANCE_COMPLETE:
            _stop_wheels(motors)
            print(f"[INFO] Mission {current_request_id} complete — returning to IDLE")
            current_request_id = None
            target_pos = None
            idle_timer = 0.0
            state = DiademState.IDLE
            print(f"[STATE] → {state} (Diadem Mission Finished)")

        elif state == DiademState.ERROR_RECOVERY:
            _stop_wheels(motors)
            state = DiademState.IDLE


def _get_position(robot, gps, imu_inertial) -> tuple[float, float, float]:
    if gps:
        try:
            vals = gps.getValues()
            if not any(math.isnan(v) for v in vals):
                return world_position(vals)
        except Exception:
            pass
    try:
        if hasattr(robot, "getSelf"):
            node = robot.getSelf()
            if node:
                p = node.getPosition()
                if p is not None and not any(math.isnan(v) for v in p):
                    return (float(p[0]), float(p[1]), float(p[2]))
    except Exception:
        pass
    return (0.0, 0.0, 0.0)


def _get_heading(robot, compass, inertial_unit) -> float:
    try:
        if hasattr(robot, "getSelf"):
            node = robot.getSelf()
            if node:
                R = node.getOrientation()
                if R and len(R) == 9:
                    # In canonical mobile robot frame (local +X forward):
                    # R[0] is East (X) component, R[6] is North (Z) component in EUN
                    return math.atan2(R[0], R[6])
    except Exception:
        pass
    if compass:
        try:
            vals = compass.getValues()
            return math.atan2(vals[0], vals[2])
        except Exception:
            pass
    return 0.0


def _set_drive(motors: dict, linear: float, angular: float) -> None:
    v_left  = (linear - angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    v_right = (linear + angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS

    v_left  = clamp(v_left,  -MAX_WHEEL_VEL, MAX_WHEEL_VEL)
    v_right = clamp(v_right, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)

    # In Webots EUN with wheel axis along X:
    # Forward rotation commands:
    motors["FL"].setVelocity(v_left)
    motors["RL"].setVelocity(v_left)
    motors["FR"].setVelocity(v_right)
    motors["RR"].setVelocity(v_right)


def _stop_wheels(motors: dict) -> None:
    for m in motors.values():
        m.setVelocity(0.0)


if __name__ == "__main__":
    main()
