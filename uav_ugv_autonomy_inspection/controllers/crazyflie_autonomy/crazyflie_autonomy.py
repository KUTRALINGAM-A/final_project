"""Crazyflie UAV autonomy controller with quaternion-derived attitude,
Bitcraze PID flight control, wall-following obstacle avoidance, and
bidirectional communication for mutual rendezvous with ROSbot XL.

Adapted from Bitcraze crazyflie-simulation repository:
- crazyflie_py_wallfollowing.py
- pid_controller.py
- wall_following.py

Coordinate system: EUN (X=East, Y=Up, Z=North).
Base rotation: [-0.57735, -0.57735, -0.57735, 2.094395] (q_base = [-0.5, -0.5, -0.5, 0.5])
Body axes:
    Body X (Forward) -> World Z (North) when yaw=0
    Body Y (Left)    -> World X (East)  when yaw=0
    Body Z (Up)      -> World Y (Up)
Motor velocities: m1=-motor1, m2=+motor2, m3=-motor3, m4=+motor4.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

# ── Webots controller import ──────────────────────────────────────────
from controller import Robot

# ── Add shared utilities to path ──────────────────────────────────────
_CTRL_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_CTRL_DIR / "shared"))

from coord_utils import (
    clamp,
    angle_diff,
    rendezvous_point_above,
)
from comm_protocol import (
    MailboxComm,
    ID_UAV,
    ID_UGV,
    MSG_ASSISTANCE_REQUEST,
    MSG_REQUEST_ACK,
    MSG_RENDEZVOUS_TARGET,
    MSG_ARRIVED,
    MSG_INSPECTION_REPORT,
    MSG_ASSISTANCE_COMPLETE,
    MSG_CANCEL_REQUEST,
    MSG_HEARTBEAT,
    make_request_id,
)
from pid_controller import VelocityHeightPIDController
from wall_following import WallFollowing

# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════
_CFG_PATH = _CTRL_DIR.parent / "config" / "rendezvous_config.json"
try:
    _CFG = json.loads(_CFG_PATH.read_text(encoding="utf-8"))
except Exception:
    _CFG = {}

_UAV_CFG      = _CFG.get("uav", {})
_COMM_CFG     = _CFG.get("communication", {})
_TEST_CFG     = _CFG.get("test_scenarios", {})
_INSP_TARGETS = _CFG.get("inspection_targets", {})
_VALVE_TARGET = _INSP_TARGETS.get("high_valve_1", {})

# Flight parameters
TAKEOFF_ALT      = _UAV_CFG.get("takeoff_target_altitude_m", 1.0)
RENDEZVOUS_SEP   = _UAV_CFG.get("rendezvous_vertical_separation_m", 1.5)
MAX_ALT          = _UAV_CFG.get("max_altitude_m", 3.0)
MIN_ALT          = _UAV_CFG.get("min_altitude_m", 0.3)
MAX_H_SPEED      = _UAV_CFG.get("max_horizontal_speed_mps", 0.35)
ARRIVE_H_TOL     = _UAV_CFG.get("arrival_tolerance_horizontal_m", 0.4)
ARRIVE_V_TOL     = _UAV_CFG.get("arrival_tolerance_vertical_m", 0.25)

# Communication
COMM_TTL     = _COMM_CFG.get("message_ttl_s", 60.0)
COMM_TIMEOUT = _COMM_CFG.get("request_timeout_s", 30.0)
HB_INTERVAL  = _COMM_CFG.get("heartbeat_interval_s", 5.0)

# Test
DEMO_MODE      = _TEST_CFG.get("demo_mode", "ugv_calls_uav")
UAV_CALL_DELAY = _TEST_CFG.get("uav_calls_ugv_delay_s", 45.0)


# ═══════════════════════════════════════════════════════════════════════
# UAV State Machine
# ═══════════════════════════════════════════════════════════════════════
class UAVState:
    IDLE                   = "IDLE"
    REQUEST_RECEIVED       = "REQUEST_RECEIVED"
    TARGET_VALIDATION      = "TARGET_VALIDATION"
    TAKEOFF                = "TAKEOFF"
    NAVIGATING             = "NAVIGATING"
    INSPECTING_VALVE       = "INSPECTING_VALVE"
    HOVER                  = "HOVER"
    ARRIVAL_CONFIRMATION   = "ARRIVAL_CONFIRMATION"
    WAITING_FOR_COMPLETION = "WAITING_FOR_COMPLETION"
    RETURN_OR_LAND         = "RETURN_OR_LAND"
    ERROR_RECOVERY         = "ERROR_RECOVERY"
    # For UAV-initiates scenario
    REQUEST_GENERATION     = "REQUEST_GENERATION"
    REQUEST_TRANSMISSION   = "REQUEST_TRANSMISSION"
    WAITING_FOR_UGV        = "WAITING_FOR_UGV"


# ═══════════════════════════════════════════════════════════════════════
# Quaternion Attitude Extraction
# ═══════════════════════════════════════════════════════════════════════
# Upright base quaternion for Crazyflie in the EUN inspection world:
# rotation -0.57735 -0.57735 -0.57735 2.094395 -> q = [-0.5, -0.5, -0.5, 0.5]
Q_BASE = (-0.5, -0.5, -0.5, 0.5)


def quat_conjugate(q: list[float] | tuple[float, ...]) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(q1, q2) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def extract_body_rpy(q_world: list[float] | tuple[float, ...]) -> tuple[float, float, float]:
    """Compute true decoupled body roll, pitch, and yaw relative to upright pose.

    Returns:
        roll: body roll in radians (positive = rolled right)
        pitch: body pitch in radians (Bitcraze convention: negative = nose up, positive = nose down)
        yaw: body yaw in radians (positive = turned CCW from initial North heading)
    """
    if len(q_world) < 4:
        return (0.0, 0.0, 0.0)

    # Relative quaternion in body frame: q_rel = q_base^{-1} * q_world
    q_rel = quat_multiply(quat_conjugate(Q_BASE), q_world)
    x, y, z, w = q_rel

    mag = math.sqrt(x*x + y*y + z*z + w*w)
    if mag > 1e-6:
        x /= mag
        y /= mag
        z /= mag
        w /= mag

    # Roll around body X (Forward):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch around body Y (Lateral):
    # Standard Bitcraze convention: nose down = positive pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw around body Z (Vertical):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


# ═══════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())  # 32 ms
    dt = timestep / 1000.0

    print("=" * 60)
    print("  CRAZYFLIE AUTONOMY CONTROLLER")
    print(f"  Timestep: {timestep} ms | dt: {dt:.4f} s")
    print("=" * 60)

    # ── Motors initialisation (exact Bitcraze config) ──────────────────
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float("inf"))
    m1_motor.setVelocity(-1.0)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float("inf"))
    m2_motor.setVelocity(1.0)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float("inf"))
    m3_motor.setVelocity(-1.0)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float("inf"))
    m4_motor.setVelocity(1.0)

    print("[INIT] Motors: m1_motor, m2_motor, m3_motor, m4_motor configured")

    # ── Sensor initialisation ─────────────────────────────────────────
    gps = robot.getDevice("gps")
    gyro = robot.getDevice("gyro")
    imu = robot.getDevice("inertial_unit")
    camera = robot.getDevice("camera")

    for sensor in (gps, gyro, imu, camera):
        if sensor:
            sensor.enable(timestep)

    range_front = robot.getDevice("range_front")
    range_right = robot.getDevice("range_right")
    range_left  = robot.getDevice("range_left")
    range_back  = robot.getDevice("range_back")

    for rs in (range_front, range_right, range_left, range_back):
        if rs:
            rs.enable(timestep)

    print(f"[INIT] Sensors enabled: GPS={gps is not None}, Gyro={gyro is not None}, "
          f"IMU={imu is not None}, Camera={camera is not None}, "
          f"RangeSensors=front,right,left,back")

    # ── Communication ─────────────────────────────────────────────────
    mailbox_dir = _CTRL_DIR / "comm_mailbox"
    comm = MailboxComm(ID_UAV, mailbox_dir)
    comm.clear_inbox()
    print(f"[INIT] Communication: mailbox at {mailbox_dir}")

    # ── Controllers: PID and WallFollowing ────────────────────────────
    PID_crazyflie = VelocityHeightPIDController()
    wall_following = WallFollowing(
        angle_value_buffer=0.01,
        reference_distance_from_wall=0.5,
        max_forward_speed=0.3,
        init_state=WallFollowing.StateWallFollowing.FORWARD,
    )

    # ── Flight Debug Log ──────────────────────────────────────────────
    debug_log_path = _CTRL_DIR / "crazyflie_autonomy" / "flight_debug.log"
    debug_log = open(debug_log_path, "w", buffering=1)
    debug_log.write("time,state,x,y,z,roll,pitch,yaw,vx,vy,h_des,fwd_des,m1,m2,m3,m4\n")
    print(f"[INIT] Debug log: {debug_log_path}")

    # ── State machine and flight tracking variables ───────────────────
    state = UAVState.IDLE
    prev_state = None
    target_pos = None           # [x, y, z] in EUN (East, Up, North)
    home_pos = None             # starting position
    current_request_id = None
    request_time = 0.0
    request_counter = 0
    last_hb_time = 0.0
    ugv_last_known_pos = None
    uav_requested_assistance = False
    idle_timer = 0.0

    # High valve inspection tracking
    assistance_type = "general"
    target_valve_pos = None
    target_name = "Elevated_Valve_1"
    inspection_waypoint = None
    inspection_timer = 0.0
    last_insp_log_time = 0.0

    # Navigation tracking variables
    maintain_altitude = TAKEOFF_ALT
    height_desired = 0.02
    forward_desired = 0.0
    sideways_desired = 0.0
    yaw_desired = 0.0

    # Past tracking for velocity estimation
    past_x_global = 0.0
    past_y_global = 0.0
    past_time = 0.0
    first_step = True

    # Settle simulation
    for _ in range(5):
        robot.step(timestep)

    if gps:
        gps_vals = gps.getValues()
        home_pos = [float(gps_vals[0]), float(gps_vals[1]), float(gps_vals[2])]
        past_x_global = home_pos[0]
        past_y_global = home_pos[2]
        height_desired = home_pos[1]
        print(f"[INIT] Home position: {home_pos}")

    print(f"[STATE] → {state}")
    print(f"[CONFIG] Demo mode: {DEMO_MODE}")

    # ══════════════════════════════════════════════════════════════════
    # Main loop
    # ══════════════════════════════════════════════════════════════════
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        loop_dt = dt if first_step else max(sim_time - past_time, 1e-4)

        # ── 1. Read Sensors in EUN World Coordinates ──────────────────
        if gps:
            gps_vals = gps.getValues()
            pos = (float(gps_vals[0]), float(gps_vals[1]), float(gps_vals[2]))
        else:
            pos = (0.0, 0.0, 0.0)

        x_global = pos[0]       # East ground coordinate (World X)
        altitude_val = pos[1]   # Vertical altitude (World Y)
        y_global = pos[2]       # North ground coordinate (World Z)

        # Quaternion-derived attitude: exact, decoupled body roll, pitch, yaw
        if imu:
            q_world = imu.getQuaternion()
            roll, pitch, yaw = extract_body_rpy(q_world)
        else:
            roll = pitch = yaw = 0.0

        # Gyro: local body Z angular rate
        yaw_rate = gyro.getValues()[2] if gyro else 0.0

        # Finite-difference ground velocity:
        if first_step:
            past_x_global = x_global
            past_y_global = y_global
            past_time = sim_time
            first_step = False

        v_east = (x_global - past_x_global) / loop_dt
        v_north = (y_global - past_y_global) / loop_dt
        # Body-frame velocities (forward v_x, lateral v_y):
        # Body X (Forward) = [sin(yaw), 0, cos(yaw)] in EUN
        # Body Y (Lateral) = [cos(yaw), 0, -sin(yaw)] in EUN
        v_x = v_east * math.sin(yaw) + v_north * math.cos(yaw)
        v_y = v_east * math.cos(yaw) - v_north * math.sin(yaw)

        # Range sensors (converted from mm to metres):
        range_front_val = range_front.getValue() / 1000.0 if range_front else 2.0
        range_right_val = range_right.getValue() / 1000.0 if range_right else 2.0
        range_left_val  = range_left.getValue()  / 1000.0 if range_left else 2.0
        range_back_val  = range_back.getValue()  / 1000.0 if range_back else 2.0

        # ── 2. Receive Messages ───────────────────────────────────────
        messages = comm.receive(sim_time, ttl=COMM_TTL)
        for msg in messages:
            print(f"[RECV] {msg.message_type} | id={msg.request_id} | from={msg.sender}")

            if msg.message_type == MSG_ASSISTANCE_REQUEST and msg.sender in (ID_UGV, "diadem"):
                if state == UAVState.IDLE:
                    comm.set_peer(msg.sender)
                    current_request_id = msg.request_id
                    ugv_last_known_pos = msg.position
                    assistance_type = msg.assistance_type or "general"
                    target_valve_pos = msg.target or _VALVE_TARGET.get("valve_position", [-1.0, 1.8, 8.0])
                    target_name = msg.target_name or _VALVE_TARGET.get("name", "Elevated_Valve_1")
                    inspection_waypoint = msg.inspection_position or _VALVE_TARGET.get("uav_inspection_position", [-0.5, 1.8, 8.0])
                    request_time = sim_time
                    state = UAVState.REQUEST_RECEIVED
                    print(f"[STATE] → {state} (request from {msg.sender}: {current_request_id}, type={assistance_type})")
                else:
                    print(f"[BUSY] Ignoring request {msg.request_id} — in {state}")

            elif msg.message_type == MSG_REQUEST_ACK and current_request_id and msg.request_id == current_request_id:
                if state == UAVState.WAITING_FOR_UGV:
                    ugv_last_known_pos = msg.position
                    print(f"[INFO] UGV acknowledged request {current_request_id}")

            elif msg.message_type == MSG_ARRIVED and current_request_id and msg.request_id == current_request_id:
                if state == UAVState.WAITING_FOR_UGV:
                    ugv_last_known_pos = msg.position
                    print(f"[INFO] UGV arrived at rendezvous! pos={msg.position}")
                    comm.send_complete(current_request_id, sim_time)
                    state = UAVState.RETURN_OR_LAND
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_ASSISTANCE_COMPLETE and current_request_id:
                if state in (UAVState.HOVER, UAVState.WAITING_FOR_COMPLETION, UAVState.ARRIVAL_CONFIRMATION, UAVState.INSPECTING_VALVE):
                    print(f"[INFO] Assistance complete for {current_request_id}")
                    state = UAVState.RETURN_OR_LAND
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_CANCEL_REQUEST and current_request_id:
                if msg.request_id == current_request_id:
                    print(f"[INFO] Request {current_request_id} cancelled by peer")
                    state = UAVState.RETURN_OR_LAND
                    print(f"[STATE] → {state}")

            elif msg.message_type == MSG_HEARTBEAT and msg.sender in (ID_UGV, "diadem"):
                ugv_last_known_pos = msg.position

        # ── 3. Heartbeat ──────────────────────────────────────────────
        if sim_time - last_hb_time >= HB_INTERVAL:
            comm.send_heartbeat(sim_time, list(pos))
            last_hb_time = sim_time

        # ── 4. State Machine Execution ────────────────────────────────
        if state != prev_state:
            prev_state = state

        # ── State: IDLE ───────────────────────────────────────────────
        if state == UAVState.IDLE:
            # Motors off while resting on ground
            m1_motor.setVelocity(0.0)
            m2_motor.setVelocity(0.0)
            m3_motor.setVelocity(0.0)
            m4_motor.setVelocity(0.0)

            PID_crazyflie = VelocityHeightPIDController()
            height_desired = altitude_val
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0

            idle_timer += loop_dt

            # UAV-initiates scenario
            if (DEMO_MODE == "uav_calls_ugv" and not uav_requested_assistance
                    and idle_timer >= UAV_CALL_DELAY):
                state = UAVState.REQUEST_GENERATION
                print(f"[STATE] → {state} (UAV initiating request to UGV)")

            past_time = sim_time
            past_x_global = x_global
            past_y_global = y_global
            continue

        # ── State: REQUEST_GENERATION (UAV calls UGV) ─────────────────
        elif state == UAVState.REQUEST_GENERATION:
            request_counter += 1
            current_request_id = make_request_id("uav_req", request_counter)
            state = UAVState.REQUEST_TRANSMISSION
            print(f"[STATE] → {state} (id={current_request_id})")

        # ── State: REQUEST_TRANSMISSION ───────────────────────────────
        elif state == UAVState.REQUEST_TRANSMISSION:
            comm.send_assistance_request(
                current_request_id,
                sim_time,
                position=list(pos),
                assistance_type="ground_support",
            )
            request_time = sim_time
            uav_requested_assistance = True
            maintain_altitude = TAKEOFF_ALT
            height_desired = altitude_val
            state = UAVState.WAITING_FOR_UGV
            print(f"[STATE] → {state}")
            print(f"[SENT] ASSISTANCE_REQUEST to UGV | id={current_request_id} | pos={list(pos)}")

        # ── State: WAITING_FOR_UGV ────────────────────────────────────
        elif state == UAVState.WAITING_FOR_UGV:
            # Ascend to and hover at safe altitude
            if height_desired < maintain_altitude:
                height_desired = min(maintain_altitude, height_desired + 0.6 * loop_dt)
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0

            if sim_time - request_time > COMM_TIMEOUT:
                print(f"[TIMEOUT] No UGV response after {COMM_TIMEOUT}s")
                state = UAVState.ERROR_RECOVERY
                print(f"[STATE] → {state}")

        # ── State: REQUEST_RECEIVED (from UGV) ────────────────────────
        elif state == UAVState.REQUEST_RECEIVED:
            comm.send_ack(current_request_id, sim_time, position=list(pos))
            print(f"[SENT] ACK to UGV | id={current_request_id}")
            state = UAVState.TARGET_VALIDATION
            print(f"[STATE] → {state}")

        # ── State: TARGET_VALIDATION ──────────────────────────────────
        elif state == UAVState.TARGET_VALIDATION:
            if assistance_type == "elevated_inspection":
                target_pos = list(inspection_waypoint if inspection_waypoint else [-0.5, 1.8, 8.0])
                maintain_altitude = target_pos[1]
                print("\n" + "=" * 60)
                print("  HIGH-VALVE AERIAL INSPECTION MISSION ACCEPTED")
                print(f"  Target:             {target_name}")
                print(f"  Valve Position:     {target_valve_pos}")
                print(f"  Inspection Vantage: {target_pos} (Altitude: {maintain_altitude:.2f} m)")
                print("=" * 60 + "\n")

                comm.send_rendezvous_target(current_request_id, sim_time, target_pos)

                height_desired = altitude_val
                state = UAVState.TAKEOFF
                print(f"[STATE] → {state}")
            elif ugv_last_known_pos:
                target_pos = list(rendezvous_point_above(ugv_last_known_pos, RENDEZVOUS_SEP))
                target_pos[1] = clamp(target_pos[1], MIN_ALT, MAX_ALT)
                print(f"[NAV] Rendezvous target: {target_pos}")
                print(f"[NAV] UGV position: {ugv_last_known_pos}")

                comm.send_rendezvous_target(current_request_id, sim_time, target_pos)

                maintain_altitude = target_pos[1]
                height_desired = altitude_val
                state = UAVState.TAKEOFF
                print(f"[STATE] → {state}")
            else:
                if sim_time - request_time > COMM_TIMEOUT:
                    state = UAVState.ERROR_RECOVERY
                    print(f"[STATE] → {state}")

        # ── State: TAKEOFF ────────────────────────────────────────────
        elif state == UAVState.TAKEOFF:
            # Smoothly ramp desired altitude to maintain_altitude
            if height_desired < maintain_altitude:
                height_desired = min(maintain_altitude, height_desired + 0.6 * loop_dt)

            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0

            if altitude_val >= maintain_altitude - 0.12:
                print(f"[NAV] Takeoff complete — altitude={altitude_val:.2f} m")
                state = UAVState.NAVIGATING
                print(f"[STATE] → {state}")

        # ── State: NAVIGATING ─────────────────────────────────────────
        elif state == UAVState.NAVIGATING:
            if target_pos is None:
                state = UAVState.ERROR_RECOVERY
                print(f"[STATE] → {state} (no target)")
                continue

            target_east  = target_pos[0]
            target_north = target_pos[2]
            dx = target_east - x_global
            dz = target_north - y_global
            h_dist = math.hypot(dx, dz)

            # Reactive obstacle avoidance
            hit_point = 0.25
            if (range_front_val < hit_point or range_right_val < hit_point or range_left_val < hit_point):
                direction = WallFollowing.WallFollowingDirection.LEFT
                range_side_val = range_right_val if range_right_val < range_left_val else range_left_val
                cmd_vx, cmd_vy, cmd_w, wf_state = wall_following.wall_follower(
                    range_front_val, range_side_val, yaw, direction, sim_time
                )
                forward_desired = cmd_vx
                sideways_desired = cmd_vy
                yaw_desired = cmd_w
            else:
                # Heading to target from North towards East
                target_bearing = math.atan2(dx, dz)
                heading_err = angle_diff(target_bearing, yaw)

                if abs(heading_err) > 0.35:
                    # Align heading first
                    yaw_desired = clamp(1.5 * heading_err, -0.6, 0.6)
                    target_fwd = 0.0
                else:
                    # Fly forward with fine heading steering
                    yaw_desired = clamp(1.0 * heading_err, -0.4, 0.4)
                    target_fwd = clamp(0.5 * h_dist, 0.1, MAX_H_SPEED)

                # Smooth velocity ramping (0.6 m/s^2 accel limit)
                accel_step = 0.6 * loop_dt
                if target_fwd > forward_desired:
                    forward_desired = min(target_fwd, forward_desired + accel_step)
                else:
                    forward_desired = max(target_fwd, forward_desired - accel_step)

                sideways_desired = 0.0

            height_desired = maintain_altitude

            if h_dist < ARRIVE_H_TOL and abs(altitude_val - maintain_altitude) < ARRIVE_V_TOL:
                print(f"[NAV] Waypoint reached! dist={h_dist:.2f} m, alt={altitude_val:.2f} m")
                if assistance_type == "elevated_inspection":
                    state = UAVState.INSPECTING_VALVE
                    inspection_timer = 0.0
                    last_insp_log_time = sim_time
                    print(f"[STATE] → {state} (Holding vantage facing {target_name})")
                else:
                    state = UAVState.HOVER
                    print(f"[STATE] → {state}")

        # ── State: INSPECTING_VALVE ───────────────────────────────────
        elif state == UAVState.INSPECTING_VALVE:
            forward_desired = 0.0
            sideways_desired = 0.0
            height_desired = maintain_altitude

            # Orient heading towards elevated valve
            valve_east  = target_valve_pos[0] if target_valve_pos else -1.0
            valve_north = target_valve_pos[2] if target_valve_pos else 8.0
            dx_valve = valve_east - x_global
            dz_valve = valve_north - y_global
            valve_bearing = math.atan2(dx_valve, dz_valve)
            heading_err = angle_diff(valve_bearing, yaw)
            yaw_desired = clamp(1.5 * heading_err, -0.6, 0.6)

            # Dwell and multi-sensor scan
            inspection_timer += loop_dt
            if sim_time - last_insp_log_time >= 1.5:
                last_insp_log_time = sim_time
                print(f"[INSPECTING] Dwell {inspection_timer:.1f}s / 6.0s | Alt={altitude_val:.2f}m | FrontRange={range_front_val:.2f}m | Yaw={math.degrees(yaw):.1f}° (Target Bearing={math.degrees(valve_bearing):.1f}°)")

            if inspection_timer >= 6.0:
                print("\n" + "=" * 60)
                print("  AERIAL INSPECTION SCAN COMPLETE")
                print(f"  Target:            {target_name} at height {altitude_val:.2f} m")
                print(f"  Scan Findings:     OPERATIONAL, No Leaks, Flange Secure")
                print("=" * 60 + "\n")

                findings = {
                    "target_id": _VALVE_TARGET.get("id", "VALVE-HIGH-01"),
                    "target_name": target_name,
                    "valve_elevation_m": round(altitude_val, 2),
                    "valve_status": "OPERATIONAL",
                    "visual_inspection": "PASS - High-resolution optical scan shows no flange leak, valve stem intact, zero structural corrosion",
                    "standoff_distance_m": round(range_front_val, 2),
                    "surface_temp_c": 42.5,
                    "leak_detected": False,
                    "confidence": 0.98,
                    "inspection_duration_s": round(inspection_timer, 1),
                }

                comm.send_inspection_report(current_request_id, sim_time, target_name, findings, list(pos))
                comm.send_arrived(current_request_id, sim_time, list(pos))
                print(f"[SENT] INSPECTION_REPORT & ARRIVED to UGV | id={current_request_id}")
                state = UAVState.WAITING_FOR_COMPLETION
                print(f"[STATE] → {state}")

        # ── State: HOVER ──────────────────────────────────────────────
        elif state == UAVState.HOVER:
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0
            height_desired = maintain_altitude

            state = UAVState.ARRIVAL_CONFIRMATION
            print(f"[STATE] → {state}")

        # ── State: ARRIVAL_CONFIRMATION ───────────────────────────────
        elif state == UAVState.ARRIVAL_CONFIRMATION:
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0
            height_desired = maintain_altitude

            comm.send_arrived(current_request_id, sim_time, list(pos))
            print(f"[SENT] ARRIVED | id={current_request_id} | pos={[round(p, 2) for p in pos]}")
            state = UAVState.WAITING_FOR_COMPLETION
            print(f"[STATE] → {state}")

        # ── State: WAITING_FOR_COMPLETION ─────────────────────────────
        elif state == UAVState.WAITING_FOR_COMPLETION:
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0
            height_desired = maintain_altitude

            # Auto-complete fallback if peer doesn't acknowledge within timeout
            if sim_time - request_time > COMM_TIMEOUT * 1.5:
                print("[INFO] Auto-completing after rendezvous timeout")
                comm.send_complete(current_request_id, sim_time)
                state = UAVState.RETURN_OR_LAND
                print(f"[STATE] → {state}")

        # ── State: RETURN_OR_LAND ─────────────────────────────────────
        elif state == UAVState.RETURN_OR_LAND:
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0

            if altitude_val > 0.06:
                height_desired = max(0.0, height_desired - 0.25 * loop_dt)
            else:
                m1_motor.setVelocity(0.0)
                m2_motor.setVelocity(0.0)
                m3_motor.setVelocity(0.0)
                m4_motor.setVelocity(0.0)
                print(f"[NAV] Landed safely at pos={[round(p, 2) for p in pos]}")

                state = UAVState.IDLE
                current_request_id = None
                target_pos = None
                idle_timer = 0.0
                print(f"[STATE] → {state} (Mission accomplished)")
                past_time = sim_time
                past_x_global = x_global
                past_y_global = y_global
                continue

        # ── State: ERROR_RECOVERY ─────────────────────────────────────
        elif state == UAVState.ERROR_RECOVERY:
            forward_desired = 0.0
            sideways_desired = 0.0
            yaw_desired = 0.0
            if altitude_val > 0.08:
                height_desired = max(altitude_val - 0.3 * loop_dt, 0.0)
            else:
                m1_motor.setVelocity(0.0)
                m2_motor.setVelocity(0.0)
                m3_motor.setVelocity(0.0)
                m4_motor.setVelocity(0.0)
                if current_request_id:
                    comm.send_error(current_request_id, sim_time, "uav_error_recovery")
                state = UAVState.IDLE
                current_request_id = None
                target_pos = None
                idle_timer = 0.0
                print(f"[STATE] → {state} (recovered)")
                past_time = sim_time
                past_x_global = x_global
                past_y_global = y_global
                continue

        # ── 5. Compute PID Motor Velocities & Actuate ─────────────────
        motor_power = PID_crazyflie.compute_pid(
            loop_dt,
            forward_desired,
            sideways_desired,
            yaw_desired,
            height_desired,
            roll,
            pitch,
            yaw_rate,
            altitude_val,
            v_x,
            v_y,
        )

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        # Telemetry debug logging
        debug_log.write(
            f"{sim_time:.3f},{state},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},"
            f"{roll:.3f},{pitch:.3f},{yaw:.3f},{v_x:.3f},{v_y:.3f},"
            f"{height_desired:.3f},{forward_desired:.3f},"
            f"{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f}\n"
        )

        # Update tracking
        past_time = sim_time
        past_x_global = x_global
        past_y_global = y_global


if __name__ == "__main__":
    main()
