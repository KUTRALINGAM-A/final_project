"""
infinity_controller.py
----------------------
Webots Python Controller for a 4-wheel differential/skid-steering UGV (Husarion ROSbot XL)
that continuously follows a parametric infinity (∞) / figure-eight trajectory:

    x(t) = A * sin(omega * t)
    y(t) = B * sin(2 * omega * t)

Kinematics:
    v = sqrt(x_dot^2 + y_dot^2)
    yaw_rate = (x_dot * y_ddot - y_dot * x_ddot) / (x_dot^2 + y_dot^2)
    v_left  = v - (track_width / 2) * yaw_rate
    v_right = v + (track_width / 2) * yaw_rate
    omega_left  = v_left / wheel_radius
    omega_right = v_right / wheel_radius
"""

import math
import sys
import os

try:
    from controller import Supervisor as RobotController
    HAS_SUPERVISOR = True
except ImportError:
    from controller import Robot as RobotController
    HAS_SUPERVISOR = False

# ── ROS 2 Integration for UGV (Callsign: Charlie) ──
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from std_msgs.msg import String as RosString
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class UgvCharlieNode(RosNode):
    """ROS 2 Node representing the UGV (Charlie) communicating with UAV (Tiger)."""
    def __init__(self):
        super().__init__('ugv_node')
        self.pub_to_tiger = self.create_publisher(RosString, '/charlie_to_tiger', 10)
        self.sub_from_tiger = self.create_subscription(
            RosString, '/tiger_to_charlie', self.tiger_callback, 10
        )
        self.handshake_complete = False
        self.get_logger().info('🚜 [Charlie / UGV Node] Online on ROS 2.')
        print('🚜 [Charlie / UGV Node] Online and listening on ROS 2.', flush=True)

    def tiger_callback(self, msg):
        text = msg.data.strip()
        banner = f"\n{'='*48}\n🚜 [Charlie / UGV] RECEIVED from Tiger:\n{text}\n{'='*48}"
        print(banner, flush=True)
        self.get_logger().info(f"[Charlie <- Tiger] RECEIVED:\n{text}")

        if "tiger in" in text and not self.handshake_complete:
            reply_text = "charlie to tiger\nmessage received\nmessage received"
            reply_msg = RosString()
            reply_msg.data = reply_text
            self.pub_to_tiger.publish(reply_msg)
            reply_banner = f"\n{'='*48}\n🚜 [Charlie / UGV] TRANSMITTING to Tiger:\n{reply_text}\n{'='*48}"
            print(reply_banner, flush=True)
            self.get_logger().info(f"[Charlie -> Tiger] TRANSMITTING:\n{reply_text}")
        elif "communication successful" in text:
            confirm_msg = "✅ 🚜 [Charlie / UGV] Final confirmation received. Communication channel established successfully!\n"
            print(confirm_msg, flush=True)
            self.get_logger().info("✅ [Charlie] Final confirmation received. Communication channel established successfully!")
            self.handshake_complete = True

# ---------------------------------------------------------------------------
# Configuration Parameters
# ---------------------------------------------------------------------------
# ROSbot XL dimensions: 100mm diameter wheels -> radius 0.05m, track width ~0.27m
WHEEL_RADIUS = 0.05          # Wheel radius in meters
TRACK_WIDTH  = 0.27          # Distance between left and right wheels in meters

A = 2.0                      # Trajectory width half-extent (meters)
B = 1.0                      # Trajectory height half-extent (meters)
OMEGA = 0.25                 # Trajectory angular frequency parameter (rad/s)
X_OFFSET = 26.0              # Trajectory center X (near the drone at X=29.29, Y=-10.46)
Y_OFFSET = -11.0             # Trajectory center Y

MAX_WHEEL_ANGULAR_VEL = 20.0 # Maximum wheel angular velocity limit (rad/s)
SMOOTHING_ALPHA = 0.25       # Smoothing filter coefficient (0 < alpha <= 1)
EPS = 1e-6                   # Numerical threshold to avoid zero division

# Closed-loop tracking gains (ensures robust, indefinite figure-eight tracking
# despite realistic physical wheel slip and skid-steer tire friction)
K_LONG = 1.2                 # Longitudinal speed correction gain
K_THETA = 2.8                # Heading orientation correction gain
K_LAT = 1.5                  # Lateral cross-track error correction gain

TRAIL_SAMPLE_PERIOD = 0.15   # Interval (seconds) between path trail point additions
TRAIL_MAX_POINTS = 5000      # Maximum points retained in the trail visualizer

# Motor naming candidates: supports ROSbot XL joints and generic UGV motors
MOTOR_CANDIDATES = {
    "front_left":  ["fl_wheel_joint", "front_left_motor", "fl_wheel_motor", "wheel1"],
    "rear_left":   ["rl_wheel_joint", "rear_left_motor", "rl_wheel_motor", "wheel3"],
    "front_right": ["fr_wheel_joint", "front_right_motor", "fr_wheel_motor", "wheel2"],
    "rear_right":  ["rr_wheel_joint", "rear_right_motor", "rr_wheel_motor", "wheel4"],
}


def trajectory_state(t):
    """
    Computes desired position and 1st and 2nd time derivatives for:
        x(t) = A * sin(omega * t)
        y(t) = B * sin(2 * omega * t)
    """
    w = OMEGA
    wt = w * t

    x = X_OFFSET + A * math.sin(wt)
    y = Y_OFFSET + B * math.sin(2.0 * wt)

    x_dot = A * w * math.cos(wt)
    y_dot = 2.0 * B * w * math.cos(2.0 * wt)

    x_ddot = -A * (w ** 2) * math.sin(wt)
    y_ddot = -4.0 * B * (w ** 2) * math.sin(2.0 * wt)

    return x, y, x_dot, y_dot, x_ddot, y_ddot


def compute_feedforward_twist(x_dot, y_dot, x_ddot, y_ddot):
    """
    Computes nominal linear velocity v and angular yaw velocity (yaw_rate) from derivatives:
        v = sqrt(x_dot^2 + y_dot^2)
        yaw_rate = (x_dot * y_ddot - y_dot * x_ddot) / (x_dot^2 + y_dot^2)
    """
    speed_sq = x_dot * x_dot + y_dot * y_dot
    v = math.sqrt(speed_sq)
    denom = max(speed_sq, EPS)
    yaw_rate = (x_dot * y_ddot - y_dot * x_ddot) / denom
    desired_heading = math.atan2(y_dot, x_dot)
    return v, yaw_rate, desired_heading


def angle_diff(target, current):
    """Normalized angular difference in [-pi, pi]."""
    diff = target - current
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


class TrailDrawer:
    """
    Dynamically appends points to the IndexedLineSet DEF TRAIL_LINE_SET in Webots
    to visualize the ground path traveled by the UGV in real time.
    """
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.enabled = False
        if not hasattr(supervisor, "getFromDef"):
            return

        line_set_node = supervisor.getFromDef("TRAIL_LINE_SET")
        self.enabled = line_set_node is not None
        if not self.enabled:
            print("[TrailDrawer] Note: TRAIL_LINE_SET DEF not found in world. Path visualization disabled.")
            return

        coord_node = line_set_node.getField("coord").getSFNode()
        self.point_field = coord_node.getField("point")
        self.coord_index_field = line_set_node.getField("coordIndex")
        self.num_points = self.point_field.getCount()

    def add_point(self, pos):
        if not self.enabled or self.num_points >= TRAIL_MAX_POINTS:
            return
        # Elevate slightly to 0.025m above ground to prevent z-fighting with the floor
        draw_pos = [pos[0], pos[1], 0.025]
        self.point_field.insertMFVec3f(-1, draw_pos)
        if self.num_points > 0:
            self.coord_index_field.insertMFInt32(-1, self.num_points - 1)
            self.coord_index_field.insertMFInt32(-1, self.num_points)
            self.coord_index_field.insertMFInt32(-1, -1)
        self.num_points += 1


def main():
    global WHEEL_RADIUS, TRACK_WIDTH

    # 1. Initialize Robot or Supervisor controller
    supervisor = RobotController()
    timestep_ms = int(supervisor.getBasicTimeStep())
    dt = timestep_ms / 1000.0

    # 2. Discover and configure motors
    motors = {}
    is_custom_ugv = False
    for pos_key, candidates in MOTOR_CANDIDATES.items():
        for name in candidates:
            motor = supervisor.getDevice(name)
            if motor is not None:
                motor.setPosition(float("inf"))  # Velocity control mode
                motor.setVelocity(0.0)
                motors[pos_key] = motor
                if name == "front_left_motor":
                    is_custom_ugv = True
                break

    # Adjust geometry if custom larger UGV is detected instead of ROSbot XL
    if is_custom_ugv:
        WHEEL_RADIUS = 0.10
        TRACK_WIDTH  = 0.40

    if len(motors) < 4:
        raise RuntimeError(f"Could not find all 4 motors! Found: {list(motors.keys())}")

    print("==================================================")
    print("  4-Wheel UGV Figure-Eight (∞) Trajectory Controller")
    print(f"  Timestep: {timestep_ms} ms | Track width: {TRACK_WIDTH:.2f} m | Wheel radius: {WHEEL_RADIUS:.2f} m")
    print(f"  A = {A:.2f} m, B = {B:.2f} m, OMEGA = {OMEGA:.2f} rad/s")
    print(f"  Motors configured: {list(motors.keys())}")
    print("==================================================")

    # 3. Pose tracking setup (Supervisor ground truth or odometry fallback)
    ugv_node = None
    translation_field = None
    rotation_field = None

    if HAS_SUPERVISOR and hasattr(supervisor, "getFromDef"):
        try:
            ugv_node = supervisor.getFromDef("UGV")
            if ugv_node is None and hasattr(supervisor, "getSelf"):
                ugv_node = supervisor.getSelf()
            if ugv_node is not None:
                translation_field = ugv_node.getField("translation")
                rotation_field = ugv_node.getField("rotation")
        except Exception as e:
            print(f"[infinity_controller] Note on supervisor pose access: {e}")
            ugv_node = None

    has_ground_truth = (ugv_node is not None and translation_field is not None)
    if has_ground_truth:
        print("[infinity_controller] Ground-truth supervisor tracking active.")
    else:
        print("[infinity_controller] Dead-reckoning odometry mode active.")

    # 4. Initialize Trail Visualizer
    trail = TrailDrawer(supervisor)
    time_since_trail_sample = TRAIL_SAMPLE_PERIOD

    # Filtered wheel speeds for smooth acceleration
    filtered_omega_left = 0.0
    filtered_omega_right = 0.0

    # Dead-reckoning state for fallback
    # Start at trajectory origin facing 45 degrees (pi/4) matching trajectory at t=0
    odom_x = X_OFFSET
    odom_y = Y_OFFSET
    odom_theta = math.pi / 4.0

    step_count = 0

    # ── Initialize ROS 2 Node for UGV (Charlie) ──
    ugv_ros = None
    if HAS_ROS2:
        try:
            rclpy.init(args=None)
            ugv_ros = UgvCharlieNode()
            print("📡 [ROS 2] UGV node (Charlie) initialized.", flush=True)
        except Exception as e:
            print(f"⚠️ [ROS 2] UGV node init notice: {e}", flush=True)

    # Main Simulation Loop
    while supervisor.step(timestep_ms) != -1:
        # Step ROS 2 callbacks and communication
        if ugv_ros is not None:
            rclpy.spin_once(ugv_ros, timeout_sec=0.001)
        t = supervisor.getTime()
        step_count += 1

        # Step 1: Calculate desired trajectory derivatives at time t
        x_des, y_des, x_dot, y_dot, x_ddot, y_ddot = trajectory_state(t)

        # Step 2 & 3: Compute desired linear velocity and yaw rate
        v_ff, yaw_rate_ff, theta_des = compute_feedforward_twist(x_dot, y_dot, x_ddot, y_ddot)

        # Read actual robot position and orientation
        if has_ground_truth:
            pos = translation_field.getSFVec3f()
            rot_mat = ugv_node.getOrientation()
            theta_actual = math.atan2(rot_mat[3], rot_mat[0])

            # If the robot was placed far from the trajectory origin on startup, reset it to the trajectory origin
            if step_count == 1 and math.hypot(pos[0] - X_OFFSET, pos[1] - Y_OFFSET) > 2.5:
                print(f"[infinity_controller] Repositioning robot from ({pos[0]:.2f}, {pos[1]:.2f}) to trajectory origin ({X_OFFSET:.2f}, {Y_OFFSET:.2f}).")
                translation_field.setSFVec3f([X_OFFSET, Y_OFFSET, 0.05])
                if rotation_field is not None:
                    rotation_field.setSFRotation([0.0, 0.0, 1.0, math.pi / 4.0])
                if hasattr(ugv_node, "resetPhysics"):
                    ugv_node.resetPhysics()
                pos = [X_OFFSET, Y_OFFSET, 0.05]
                theta_actual = math.pi / 4.0
        else:
            # Integrate dead-reckoning odometry from commanded wheel velocities
            v_est = (filtered_omega_left + filtered_omega_right) * 0.5 * WHEEL_RADIUS
            yaw_rate_est = (filtered_omega_right - filtered_omega_left) * WHEEL_RADIUS / TRACK_WIDTH
            odom_theta += yaw_rate_est * dt
            odom_x += v_est * math.cos(odom_theta) * dt
            odom_y += v_est * math.sin(odom_theta) * dt
            pos = [odom_x, odom_y, 0.0]
            theta_actual = odom_theta

        # Tracking errors in trajectory Frenet frame
        dx = x_des - pos[0]
        dy = y_des - pos[1]
        e_long = math.cos(theta_des) * dx + math.sin(theta_des) * dy
        e_lat  = -math.sin(theta_des) * dx + math.cos(theta_des) * dy
        e_theta = angle_diff(theta_des, theta_actual)

        # Combined feedforward + feedback control
        v = v_ff + K_LONG * e_long
        v = max(0.05, v)  # ensure continuous forward motion

        yaw_rate = yaw_rate_ff + K_THETA * e_theta + K_LAT * e_lat

        # Step 4: Differential-drive / skid-steering wheel linear velocities
        v_left  = v - (TRACK_WIDTH / 2.0) * yaw_rate
        v_right = v + (TRACK_WIDTH / 2.0) * yaw_rate

        # Convert linear wheel velocities to angular velocities (rad/s)
        omega_left_target  = v_left / WHEEL_RADIUS
        omega_right_target = v_right / WHEEL_RADIUS

        # Enforce maximum wheel speed limits
        omega_left_target  = clamp(omega_left_target,  -MAX_WHEEL_ANGULAR_VEL, MAX_WHEEL_ANGULAR_VEL)
        omega_right_target = clamp(omega_right_target, -MAX_WHEEL_ANGULAR_VEL, MAX_WHEEL_ANGULAR_VEL)

        # Low-pass filter to eliminate sudden jerks or acceleration spikes
        filtered_omega_left  += SMOOTHING_ALPHA * (omega_left_target  - filtered_omega_left)
        filtered_omega_right += SMOOTHING_ALPHA * (omega_right_target - filtered_omega_right)

        # Step 5 & 6: Send velocities to motors
        # Left wheels rotate together
        motors["front_left"].setVelocity(filtered_omega_left)
        motors["rear_left"].setVelocity(filtered_omega_left)

        # Right wheels rotate together
        motors["front_right"].setVelocity(filtered_omega_right)
        motors["rear_right"].setVelocity(filtered_omega_right)

        # Step 7: Update visible trail
        time_since_trail_sample += dt
        if time_since_trail_sample >= TRAIL_SAMPLE_PERIOD:
            time_since_trail_sample = 0.0
            trail.add_point(pos)

        # Periodic telemetry
        if step_count % 125 == 0:  # every ~2.0 seconds
            print(f"[t={t:5.2f}s] UGV Pos: ({pos[0]:6.3f}, {pos[1]:6.3f}) | Desired: ({x_des:6.3f}, {y_des:6.3f}) | Err: dPos={math.hypot(dx, dy):5.3f}m, eLat={e_lat:5.3f}m | wL={filtered_omega_left:5.2f}, wR={filtered_omega_right:5.2f}")

    # ── Shutdown ROS 2 Node ──
    if ugv_ros is not None:
        ugv_ros.destroy_node()
        rclpy.shutdown()
        print("📡 [ROS 2] UGV node shutdown.")


if __name__ == "__main__":
    main()
