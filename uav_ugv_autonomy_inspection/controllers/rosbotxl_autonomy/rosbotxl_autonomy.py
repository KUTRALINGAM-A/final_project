"""ROSbot XL UGV autonomy controller with differential-drive navigation,
LiDAR obstacle avoidance, and bidirectional communication for mutual
rendezvous.

Coordinate system: EUN (X=east, Y=up, Z=north).
Ground motion operates on the X-Z plane.

The ROSbot XL has 4 wheels in a skid-steer (differential-drive-like)
configuration:
    fl_wheel_joint   fr_wheel_joint
    rl_wheel_joint   rr_wheel_joint

For skid-steer turning:
    Left wheels  = fl + rl
    Right wheels = fr + rr
    Forward: all positive
    Turn left:  right forward, left backward
    Turn right: left forward, right backward
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
    ID_UGV,
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

# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════
_CFG_PATH = _CTRL_DIR.parent / "config" / "rendezvous_config.json"
try:
    _CFG = json.loads(_CFG_PATH.read_text(encoding="utf-8"))
except Exception:
    _CFG = {}

_UGV_CFG      = _CFG.get("ugv", {})
_COMM_CFG     = _CFG.get("communication", {})
_TEST_CFG     = _CFG.get("test_scenarios", {})
_NAV_CFG      = _UGV_CFG.get("navigation", {})
_INSP_TARGETS = _CFG.get("inspection_targets", {})
_CAPABILITIES = _CFG.get("capabilities", {})
_VALVE_TARGET = _INSP_TARGETS.get("high_valve_1", {})

# Drive parameters
MAX_SPEED     = _UGV_CFG.get("max_speed_mps", 0.35)
MAX_WHEEL_VEL = _UGV_CFG.get("max_wheel_speed_radps", 10.0)
WHEEL_RADIUS  = _UGV_CFG.get("wheel_radius_m", 0.05)
TRACK_WIDTH   = _UGV_CFG.get("track_width_m", 0.27)
ARRIVE_TOL    = _UGV_CFG.get("arrival_tolerance_m", 0.4)
APPROACH_SLOW = _UGV_CFG.get("approach_slowdown_distance_m", 1.0)
OBS_STOP_DIST = _UGV_CFG.get("obstacle_stop_distance_m", 0.35)
OBS_SLOW_DIST = _UGV_CFG.get("obstacle_slowdown_distance_m", 1.0)
TARGET_DEADBAND = _UGV_CFG.get("target_update_deadband_m", 0.5)

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
DEMO_MODE       = _TEST_CFG.get("demo_mode", "ugv_calls_uav")
UGV_CALL_DELAY  = _TEST_CFG.get("ugv_calls_uav_delay_s", 5.0)


# ═══════════════════════════════════════════════════════════════════════
# UGV State Machine
# ═══════════════════════════════════════════════════════════════════════
class UGVState:
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
    """Simple sector-based obstacle detection from LiDAR range data."""

    NUM_SECTORS = 12  # 30° each for 360° LiDAR

    def __init__(self):
        self.sector_min = [float("inf")] * self.NUM_SECTORS

    def update(self, lidar_device) -> None:
        """Update sector distances from a Webots Lidar device."""
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
            angle = 2.0 * math.pi * i / n_points  # 0 to 2π
            sector = int(angle / (2.0 * math.pi) * self.NUM_SECTORS) % self.NUM_SECTORS
            if r < self.sector_min[sector]:
                self.sector_min[sector] = r

    def front_distance(self) -> float:
        """Minimum distance in the forward sector (≈ ±15°)."""
        return self.sector_min[0]

    def front_left_distance(self) -> float:
        return self.sector_min[1]

    def front_right_distance(self) -> float:
        return self.sector_min[self.NUM_SECTORS - 1]

    def left_distance(self) -> float:
        return min(self.sector_min[2], self.sector_min[3])

    def right_distance(self) -> float:
        return min(self.sector_min[self.NUM_SECTORS - 2], self.sector_min[self.NUM_SECTORS - 3])

    def is_path_blocked(self, threshold: float = 0.35) -> bool:
        """True if forward path is blocked within threshold."""
        return self.front_distance() < threshold

    def compute_avoidance_angular(self) -> float:
        """Return a steering bias away from obstacles.

        Positive = turn left, negative = turn right.
        """
        left_d = self.left_distance()
        right_d = self.right_distance()
        fl_d = self.front_left_distance()
        fr_d = self.front_right_distance()

        if fl_d < OBS_SLOW_DIST or fr_d < OBS_SLOW_DIST:
            # Steer away from the closer side
            if fl_d < fr_d:
                return -OBS_GAIN  # turn right
            else:
                return OBS_GAIN   # turn left
        return 0.0


# ═══════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════
def main():
    try:
        robot = Supervisor()
        print("[INIT] Running with Supervisor API")
    except Exception as exc:
        print(f"[WARN] Supervisor instantiation failed ({exc}), falling back to Robot")
        robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    print("=" * 60)
    print("  ROSBOT XL AUTONOMY CONTROLLER")
    print(f"  Timestep: {timestep} ms | dt: {dt:.4f} s")
    print("=" * 60)

    # ── Motor initialisation ──────────────────────────────────────────
    motor_names = ["fl_wheel_joint", "fr_wheel_joint",
                   "rl_wheel_joint", "rr_wheel_joint"]
    motors = {}
    for name in motor_names:
        m = robot.getDevice(name)
        if m is None:
            print(f"[FATAL] Motor {name!r} not found!")
            return
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors[name] = m
    print(f"[INIT] Motors: {list(motors.keys())}")

    # ── Sensor initialisation ─────────────────────────────────────────
    sensor_names = [
        "front left wheel motor sensor", "front right wheel motor sensor",
        "rear left wheel motor sensor", "rear right wheel motor sensor",
    ]
    wheel_sensors = {}
    for name in sensor_names:
        s = robot.getDevice(name)
        if s:
            s.enable(timestep)
            wheel_sensors[name] = s

    lidar = robot.getDevice("laser")
    if lidar:
        lidar.enable(timestep)
        try:
            lidar.enablePointCloud()
        except Exception:
            pass
        print(f"[INIT] LiDAR enabled")
    else:
        print("[WARN] LiDAR not found!")

    # IMU devices
    imu_compass = robot.getDevice("imu compass")
    imu_gyro = robot.getDevice("imu gyro")
    imu_accel = robot.getDevice("imu accelerometer")
    imu_inertial = robot.getDevice("imu inertial_unit")

    for dev in (imu_compass, imu_gyro, imu_accel, imu_inertial):
        if dev:
            dev.enable(timestep)

    # Check for GPS (ROSbot XL PROTO may not have one — we'll use the
    # supervisor field-based position or the inertial_unit for heading)
    gps = robot.getDevice("gps")
    if gps:
        gps.enable(timestep)
        print("[INIT] GPS found and enabled")
    else:
        print("[INFO] No GPS — will use supervisor position or compass for localization")

    print(f"[INIT] Compass={imu_compass is not None}, "
          f"Gyro={imu_gyro is not None}, Accel={imu_accel is not None}, "
          f"InertialUnit={imu_inertial is not None}")

    # ── Communication ─────────────────────────────────────────────────
    mailbox_dir = _CTRL_DIR / "comm_mailbox"
    comm = MailboxComm(ID_UGV, mailbox_dir)
    comm.clear_inbox()
    print(f"[INIT] Communication: mailbox at {mailbox_dir}")

    # ── Obstacle detector ─────────────────────────────────────────────
    obs_detector = LidarObstacleDetector()

    # ── State machine ─────────────────────────────────────────────────
    state = UGVState.IDLE
    prev_state = None
    target_pos = None           # [x, y, z] ground rendezvous target
    current_request_id = None
    request_time = 0.0
    request_counter = 0
    last_hb_time = 0.0
    uav_last_known_pos = None
    ugv_initiated_request = False
    idle_timer = 0.0
    arrival_report_sent = False

    # Let simulation settle
    for _ in range(5):
        robot.step(timestep)

    # Read initial position
    initial_pos = _get_position(robot, gps, imu_inertial)
    print(f"[INIT] Initial position: {initial_pos}")
    print(f"[STATE] → {state}")
    print(f"[CONFIG] Demo mode: {DEMO_MODE}")

    # ══════════════════════════════════════════════════════════════════
    # Main loop
    # ══════════════════════════════════════════════════════════════════
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()

        # ── Read position & heading ───────────────────────────────────
        pos = _get_position(robot, gps, imu_inertial)
        heading = _get_heading(robot, imu_compass, imu_inertial)

        # ── Update LiDAR ──────────────────────────────────────────────
        if lidar:
            obs_detector.update(lidar)

        # ── Receive messages ──────────────────────────────────────────
        messages = comm.receive(sim_time, ttl=COMM_TTL)
        for msg in messages:
            print(f"[RECV] {msg.message_type} | id={msg.request_id} | from={msg.sender}")

            if msg.message_type == MSG_ASSISTANCE_REQUEST and msg.sender == ID_UAV:
                if state == UGVState.IDLE:
                    current_request_id = msg.request_id
                    uav_last_known_pos = msg.position
                    request_time = sim_time
                    state = UGVState.TARGET_VALIDATION
                    print(f"[STATE] → {state} (request from UAV: {current_request_id})")
                    # Send ACK
                    comm.send_ack(current_request_id, sim_time, position=list(pos))
                    print(f"[SENT] ACK | id={current_request_id}")
                else:
                    print(f"[BUSY] Ignoring request {msg.request_id} — in {state}")

            elif msg.message_type == MSG_REQUEST_ACK and current_request_id and msg.request_id == current_request_id:
                if state == UGVState.WAITING_FOR_UAV:
                    uav_last_known_pos = msg.position
                    print(f"[INFO] UAV acknowledged our request {current_request_id}")
                    print(f"[STATE] WAITING_FOR_UAV (UAV en route, waiting for UAV to arrive)")

            elif msg.message_type == MSG_RENDEZVOUS_TARGET and current_request_id and msg.request_id == current_request_id:
                print(f"[INFO] Received rendezvous target: {msg.target}")

            elif msg.message_type == MSG_ARRIVED and msg.sender == ID_UAV:
                if current_request_id and msg.request_id == current_request_id:
                    uav_last_known_pos = msg.position
                    print(f"[INFO] UAV arrived at waypoint! pos={msg.position}")
                    if DEMO_MODE != "ugv_calls_uav_high_valve":
                        comm.send_complete(current_request_id, sim_time)
                        print(f"[SENT] ASSISTANCE_COMPLETE | id={current_request_id}")
                        state = UGVState.ASSISTANCE_COMPLETE
                        print(f"[STATE] → {state}")
                    else:
                        print(f"[INFO] UAV on station for high valve inspection. Waiting for telemetry report...")

            elif msg.message_type == MSG_INSPECTION_REPORT and msg.sender == ID_UAV:
                if current_request_id and msg.request_id == current_request_id:
                    uav_last_known_pos = msg.position
                    findings = msg.findings or {}
                    target_name = msg.target_name or "Elevated_Valve_1"
                    print("\n" + "=" * 60)
                    print("  AERIAL INSPECTION REPORT RECEIVED FROM CRAZYFLIE UAV")
                    print("=" * 60)
                    print(f"  Request ID:       {msg.request_id}")
                    print(f"  Target:           {target_name} ({findings.get('target_id', 'VALVE-HIGH-01')})")
                    print(f"  Valve Elevation:  {findings.get('valve_elevation_m', 1.8):.2f} m")
                    print(f"  Status:           {findings.get('valve_status', 'OPERATIONAL')}")
                    print(f"  Visual Analysis:  {findings.get('visual_inspection', 'PASS')}")
                    print(f"  Standoff Dist:    {findings.get('standoff_distance_m', 0.5):.2f} m")
                    print(f"  Surface Temp:     {findings.get('surface_temp_c', 42.5)} °C")
                    print(f"  Leak Detected:    {findings.get('leak_detected', False)}")
                    print(f"  Confidence:       {findings.get('confidence', 0.98) * 100:.1f}%")
                    print("=" * 60 + "\n")

                    summary_path = mailbox_dir / "inspection_summary.json"
                    try:
                        summary_data = {
                            "request_id": msg.request_id,
                            "timestamp": sim_time,
                            "target_name": target_name,
                            "findings": findings,
                            "uav_position": msg.position,
                            "ugv_position": list(pos),
                        }
                        summary_path.write_text(json.dumps(summary_data, indent=2), encoding="utf-8")
                        print(f"[LOG] Saved inspection summary to {summary_path}")
                    except Exception as exc:
                        print(f"[WARN] Failed to write inspection summary: {exc}")

                    comm.send_complete(current_request_id, sim_time)
                    print(f"[SENT] ASSISTANCE_COMPLETE | id={current_request_id}")
                    state = UGVState.ASSISTANCE_COMPLETE
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_ASSISTANCE_COMPLETE:
                if current_request_id and msg.request_id == current_request_id:
                    print(f"[INFO] Assistance complete: {current_request_id}")
                    state = UGVState.ASSISTANCE_COMPLETE
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_CANCEL_REQUEST:
                if current_request_id and msg.request_id == current_request_id:
                    print(f"[INFO] Request cancelled: {current_request_id}")
                    _stop_wheels(motors)
                    state = UGVState.IDLE
                    current_request_id = None
                    target_pos = None
                    idle_timer = 0.0
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_HEARTBEAT and msg.sender == ID_UAV:
                uav_last_known_pos = msg.position

        # ── Heartbeat ─────────────────────────────────────────────────
        if sim_time - last_hb_time >= HB_INTERVAL:
            comm.send_heartbeat(sim_time, list(pos))
            last_hb_time = sim_time

        # ══════════════════════════════════════════════════════════════
        # State machine
        # ══════════════════════════════════════════════════════════════

        # ── IDLE ──────────────────────────────────────────────────────
        if state == UGVState.IDLE:
            _stop_wheels(motors)
            idle_timer += dt

            # In "ugv_calls_uav_high_valve" demo mode, patrol to station first
            if (DEMO_MODE == "ugv_calls_uav_high_valve" and not ugv_initiated_request
                    and idle_timer >= 2.0):
                station_coord = _VALVE_TARGET.get("ugv_station_position", [0.0, 0.04, 8.0])
                target_pos = [float(station_coord[0]), float(station_coord[1]), float(station_coord[2])]
                state = UGVState.PATROL_TO_STATION
                print("\n" + "=" * 60)
                print("  UGV AUTONOMOUS MISSION: HIGH-VALVE CORRIDOR PATROL")
                print(f"  Destination Station: {target_pos}")
                print(f"  Platform Envelope: Max Reach = {_CAPABILITIES.get('ugv', {}).get('max_inspection_height_m', 0.8):.2f} m")
                print("=" * 60)
                print(f"[STATE] → {state}")

            # In "ugv_calls_uav" demo mode, after delay, generate request
            elif (DEMO_MODE == "ugv_calls_uav" and not ugv_initiated_request
                    and idle_timer >= UGV_CALL_DELAY):
                state = UGVState.REQUEST_GENERATION
                print(f"[STATE] → {state} (UGV initiating request to UAV)")

        # ── PATROL_TO_STATION ─────────────────────────────────────────
        elif state == UGVState.PATROL_TO_STATION:
            if target_pos is None:
                state = UGVState.ERROR_RECOVERY
                continue

            h_dist = horizontal_distance(pos, target_pos)

            if h_dist < ARRIVE_TOL:
                _stop_wheels(motors)
                print(f"[NAV] Arrived at High Valve Station! (dist={h_dist:.2f} m, pos={[round(p,2) for p in pos]})")
                state = UGVState.CAPABILITY_EVAL
                print(f"[STATE] → {state}")
                continue

            desired_heading = heading_to_target(pos, target_pos)
            heading_err = angle_diff(desired_heading, heading)

            front_clear = not obs_detector.is_path_blocked(OBS_STOP_DIST)
            avoidance_steer = obs_detector.compute_avoidance_angular()

            if not front_clear:
                angular = avoidance_steer * 2.0
                linear = 0.0
                print(f"[NAV] Obstacle ahead! Steering (angular={angular:.2f})")
            else:
                if abs(heading_err) > 0.4:
                    linear = 0.0
                    angular = clamp(KP_ANG * heading_err, -2.0, 2.0)
                else:
                    angular = KP_ANG * heading_err + avoidance_steer
                    alignment = max(0.0, 1.0 - abs(heading_err) / math.pi)
                    dist_factor = min(1.0, h_dist / APPROACH_SLOW)
                    linear = KP_LIN * MAX_SPEED * alignment * dist_factor

            _set_drive(motors, linear, angular)

        # ── CAPABILITY_EVAL ───────────────────────────────────────────
        elif state == UGVState.CAPABILITY_EVAL:
            _stop_wheels(motors)
            valve_elevation = _VALVE_TARGET.get("elevation_m", 1.8)
            ugv_max_reach = _CAPABILITIES.get("ugv", {}).get("max_inspection_height_m", 0.8)
            deficit = valve_elevation - ugv_max_reach

            print("\n" + "=" * 60)
            print("  CAPABILITY-BASED TASK ALLOCATION & SENSOR REACH CHECK")
            print(f"  Target:            {_VALVE_TARGET.get('name', 'Elevated_Valve_1')} (ID: {_VALVE_TARGET.get('id', 'VALVE-HIGH-01')})")
            print(f"  Valve Elevation:   {valve_elevation:.2f} m")
            print(f"  UGV Sensor Reach:  {ugv_max_reach:.2f} m")
            print(f"  Capability Deficit: +{deficit:.2f} m (BEYOND GROUND REACH)")
            print("  Decision: Delegating Aerial Inspection to Crazyflie UAV")
            print("=" * 60 + "\n")

            state = UGVState.REQUEST_GENERATION
            print(f"[STATE] → {state}")

        # ── REQUEST_GENERATION ────────────────────────────────────────
        elif state == UGVState.REQUEST_GENERATION:
            request_counter += 1
            current_request_id = make_request_id("high_valve_insp" if DEMO_MODE == "ugv_calls_uav_high_valve" else "ugv_req", request_counter)
            state = UGVState.REQUEST_TRANSMISSION
            print(f"[STATE] → {state} (id={current_request_id})")

        # ── REQUEST_TRANSMISSION ──────────────────────────────────────
        elif state == UGVState.REQUEST_TRANSMISSION:
            if DEMO_MODE == "ugv_calls_uav_high_valve":
                comm.send_assistance_request(
                    current_request_id,
                    sim_time,
                    position=list(pos),
                    assistance_type="elevated_inspection",
                    target=_VALVE_TARGET.get("valve_position", [-1.0, 1.8, 8.0]),
                    inspection_position=_VALVE_TARGET.get("uav_inspection_position", [-0.5, 1.8, 8.0]),
                    target_name=_VALVE_TARGET.get("name", "Elevated_Valve_1"),
                )
                print(f"[SENT] ASSISTANCE_REQUEST (elevated_inspection) to UAV | id={current_request_id}")
                print(f"       Valve Target={_VALVE_TARGET.get('valve_position')} | Vantage={_VALVE_TARGET.get('uav_inspection_position')}")
            else:
                comm.send_assistance_request(
                    current_request_id,
                    sim_time,
                    position=list(pos),
                    assistance_type="aerial_inspection",
                )
                print(f"[SENT] ASSISTANCE_REQUEST to UAV | id={current_request_id} | pos={[round(p,2) for p in pos]}")
            request_time = sim_time
            ugv_initiated_request = True
            state = UGVState.WAITING_FOR_UAV
            print(f"[STATE] → {state}")

        # ── WAITING_FOR_UAV ───────────────────────────────────────────
        elif state == UGVState.WAITING_FOR_UAV:
            _stop_wheels(motors)
            if sim_time - request_time > COMM_TIMEOUT:
                print(f"[TIMEOUT] No UAV response after {COMM_TIMEOUT}s")
                state = UGVState.ERROR_RECOVERY
                print(f"[STATE] → {state}")

        # ── TARGET_VALIDATION (UAV called us) ─────────────────────────
        elif state == UGVState.TARGET_VALIDATION:
            if uav_last_known_pos:
                # Ground projection of UAV position
                target_pos = list(rendezvous_point_below(uav_last_known_pos, ground_y=0.0))
                print(f"[NAV] Rendezvous target (ground proj): {target_pos}")
                print(f"[NAV] UAV position: {uav_last_known_pos}")

                h_dist = horizontal_distance(pos, target_pos)
                if h_dist < ARRIVE_TOL:
                    print(f"[NAV] Already at target (dist={h_dist:.2f})")
                    state = UGVState.STOPPED_AT_TARGET
                    print(f"[STATE] → {state}")
                else:
                    state = UGVState.PATH_PLANNING
                    print(f"[STATE] → {state}")
            else:
                print("[WARN] No UAV position — waiting")
                if sim_time - request_time > COMM_TIMEOUT:
                    state = UGVState.ERROR_RECOVERY

        # ── PATH_PLANNING ─────────────────────────────────────────────
        elif state == UGVState.PATH_PLANNING:
            # Simple direct-drive approach; obstacle avoidance via LiDAR
            if target_pos:
                print(f"[NAV] Planning path to {target_pos}")
                state = UGVState.NAVIGATING
                arrival_report_sent = False
                print(f"[STATE] → {state}")
            else:
                state = UGVState.ERROR_RECOVERY

        # ── NAVIGATING ────────────────────────────────────────────────
        elif state == UGVState.NAVIGATING:
            if target_pos is None:
                state = UGVState.ERROR_RECOVERY
                continue

            # Update target if UAV moved significantly
            if uav_last_known_pos:
                new_target = list(rendezvous_point_below(uav_last_known_pos, ground_y=0.0))
                dist_change = horizontal_distance(target_pos, new_target)
                if dist_change > TARGET_DEADBAND:
                    target_pos = new_target
                    print(f"[NAV] Target updated: {target_pos}")

            h_dist = horizontal_distance(pos, target_pos)

            if h_dist < ARRIVE_TOL * 1.5:
                state = UGVState.RENDEZVOUS_APPROACH
                print(f"[STATE] → {state} (dist={h_dist:.2f})")
                continue

            # Compute heading error
            desired_heading = heading_to_target(pos, target_pos)
            heading_err = angle_diff(desired_heading, heading)

            # Check obstacles
            front_clear = not obs_detector.is_path_blocked(OBS_STOP_DIST)
            avoidance_steer = obs_detector.compute_avoidance_angular()

            if not front_clear:
                # Stop and steer
                angular = avoidance_steer * 2.0
                linear = 0.0
                print(f"[NAV] Obstacle ahead! Steering (angular={angular:.2f})")
            else:
                # If misaligned, turn on the spot first; otherwise drive + steer
                if abs(heading_err) > 0.4:
                    linear = 0.0
                    angular = clamp(KP_ANG * heading_err, -2.0, 2.0)
                else:
                    angular = KP_ANG * heading_err + avoidance_steer
                    alignment = max(0.0, 1.0 - abs(heading_err) / math.pi)
                    dist_factor = min(1.0, h_dist / APPROACH_SLOW)
                    front_factor = min(1.0, (obs_detector.front_distance() - OBS_STOP_DIST) /
                                       max(OBS_SLOW_DIST - OBS_STOP_DIST, 0.01))
                    front_factor = max(0.0, front_factor)
                    linear = KP_LIN * MAX_SPEED * alignment * dist_factor * front_factor

            _set_drive(motors, linear, angular)

        # ── RENDEZVOUS_APPROACH ────────────────────────────────────────
        elif state == UGVState.RENDEZVOUS_APPROACH:
            if target_pos is None:
                state = UGVState.ERROR_RECOVERY
                continue

            h_dist = horizontal_distance(pos, target_pos)

            if h_dist < ARRIVE_TOL:
                _stop_wheels(motors)
                state = UGVState.STOPPED_AT_TARGET
                print(f"[STATE] → {state} (dist={h_dist:.2f})")
                continue

            # Slow approach
            desired_heading = heading_to_target(pos, target_pos)
            heading_err = angle_diff(desired_heading, heading)
            angular = KP_ANG * heading_err
            linear = min(MAX_SPEED * 0.3, h_dist * 0.5)
            alignment = max(0.0, 1.0 - abs(heading_err) / math.pi)
            linear *= alignment

            _set_drive(motors, linear, angular)

        # ── STOPPED_AT_TARGET ─────────────────────────────────────────
        elif state == UGVState.STOPPED_AT_TARGET:
            _stop_wheels(motors)

            if not arrival_report_sent:
                comm.send_arrived(current_request_id, sim_time, list(pos))
                print(f"[SENT] ARRIVED | id={current_request_id} | pos={[round(p,2) for p in pos]}")
                arrival_report_sent = True
                state = UGVState.ARRIVAL_CONFIRMATION
                print(f"[STATE] → {state}")

        # ── ARRIVAL_CONFIRMATION ──────────────────────────────────────
        elif state == UGVState.ARRIVAL_CONFIRMATION:
            _stop_wheels(motors)
            # Wait for ASSISTANCE_COMPLETE or timeout
            if sim_time - request_time > COMM_TIMEOUT * 2:
                print("[INFO] Auto-completing after timeout")
                comm.send_complete(current_request_id, sim_time)
                state = UGVState.ASSISTANCE_COMPLETE
                print(f"[STATE] → {state}")

        # ── ASSISTANCE_COMPLETE ───────────────────────────────────────
        elif state == UGVState.ASSISTANCE_COMPLETE:
            _stop_wheels(motors)
            print(f"[INFO] Mission {current_request_id} complete — returning to IDLE")
            current_request_id = None
            target_pos = None
            uav_last_known_pos = None
            idle_timer = 0.0
            arrival_report_sent = False
            state = UGVState.IDLE
            print(f"[STATE] → {state}")

        # ── ERROR_RECOVERY ────────────────────────────────────────────
        elif state == UGVState.ERROR_RECOVERY:
            _stop_wheels(motors)
            print("[ERROR] Recovery — stopped all wheels")
            if current_request_id:
                comm.send_error(current_request_id, sim_time, "ugv_error_recovery")
            current_request_id = None
            target_pos = None
            idle_timer = 0.0
            arrival_report_sent = False
            state = UGVState.IDLE
            print(f"[STATE] → {state} (recovered)")


# ═══════════════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════════════

def _get_position(robot, gps, imu_inertial) -> tuple[float, float, float]:
    """Get robot world position.

    Prefer GPS if available; otherwise use supervisor getSelf().getPosition().
    """
    if gps:
        try:
            vals = gps.getValues()
            if not any(math.isnan(v) for v in vals):
                return world_position(vals)
        except Exception:
            pass

    # Supervisor position
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
    """Get robot heading (yaw) in radians, 0 = north (+Z), positive = east (+X).

    Uses supervisor ground-truth orientation if available, else compass/IMU.
    """
    # 1. Supervisor ground truth orientation
    try:
        if hasattr(robot, "getSelf"):
            node = robot.getSelf()
            if node:
                R = node.getOrientation()
                if R and len(R) == 9:
                    # Forward vector is local X: (R[0] East, R[6] North)
                    return math.atan2(R[0], R[6])
    except Exception:
        pass

    # 2. Compass fallback
    if compass:
        try:
            vals = compass.getValues()
            return math.atan2(vals[0], vals[2])
        except Exception:
            pass

    # 3. IMU inertial unit fallback
    if inertial_unit:
        try:
            rpy = inertial_unit.getRollPitchYaw()
            return rpy[2]  # yaw
        except Exception:
            pass

    return 0.0


def _set_drive(motors: dict, linear: float, angular: float) -> None:
    """Convert (linear, angular) velocity to skid-steer wheel commands.

    linear  : forward speed (m/s)
    angular : turn rate (rad/s, positive = left turn)
    """
    # Differential drive model
    v_left  = (linear - angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    v_right = (linear + angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS

    v_left  = clamp(v_left,  -MAX_WHEEL_VEL, MAX_WHEEL_VEL)
    v_right = clamp(v_right, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)

    motors["fl_wheel_joint"].setVelocity(v_left)
    motors["rl_wheel_joint"].setVelocity(v_left)
    motors["fr_wheel_joint"].setVelocity(v_right)
    motors["rr_wheel_joint"].setVelocity(v_right)


def _stop_wheels(motors: dict) -> None:
    """Set all wheel velocities to zero."""
    for m in motors.values():
        m.setVelocity(0.0)


if __name__ == "__main__":
    main()
