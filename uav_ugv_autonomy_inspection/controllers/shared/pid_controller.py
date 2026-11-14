"""Velocity and height PID controller for Crazyflie quadcopter in Webots.

Ported from Bitcraze crazyflie-simulation repository:
https://github.com/bitcraze/crazyflie-simulation/blob/main/simulator_files/webots/controllers/crazyflie_controller_py/crazyflie_controller_py.py
"""
import numpy as np


class VelocityHeightPIDController:
    """Cascaded velocity and height PID controller for the Crazyflie."""

    def __init__(self):
        # Initialize error variables for velocity, altitude, pitch, and roll
        self.previous_velocity_x_error = 0.0
        self.previous_velocity_y_error = 0.0
        self.previous_altitude_error = 0.0
        self.previous_pitch_error = 0.0
        self.previous_roll_error = 0.0

        # Initialize the integral component for altitude control
        self.altitude_integral = 0.0

        # Define PID control gains for various control aspects
        self.control_gains = {
            "attitude_yaw_kp": 1.0,
            "attitude_yaw_kd": 0.5,
            "attitude_rp_kp": 0.5,
            "attitude_rp_kd": 0.1,
            "velocity_xy_kp": 2.0,
            "velocity_xy_kd": 0.5,
            "altitude_kp": 10.0,
            "altitude_ki": 5.0,
            "altitude_kd": 5.0,
        }

        # ── Data logging attributes ──
        self.logged_error_vx = 0.0
        self.logged_error_vy = 0.0
        self.logged_altitude_error = 0.0
        self.logged_error_pitch = 0.0
        self.logged_error_roll = 0.0
        self.logged_yaw_rate_error = 0.0
        self.logged_altitude_output = 0.0
        self.logged_pitch_target = 0.0
        self.logged_roll_target = 0.0

    def compute_pid(
        self,
        time_delta: float,
        target_vx: float,
        target_vy: float,
        target_yaw_rate: float,
        target_altitude: float,
        current_roll: float,
        current_pitch: float,
        current_yaw_rate: float,
        current_altitude: float,
        current_vx: float,
        current_vy: float,
    ) -> list[float]:
        """Compute PID outputs for controlling velocity and maintaining fixed height.

        Returns [motor1, motor2, motor3, motor4] commanded speeds in [0, 600] rad/s.
        """
        dt = max(time_delta, 1e-6)

        # ── Horizontal velocity errors & attitude targets ──────────────
        error_vx = target_vx - current_vx
        derivative_vx = (error_vx - self.previous_velocity_x_error) / dt

        error_vy = target_vy - current_vy
        derivative_vy = (error_vy - self.previous_velocity_y_error) / dt

        # Clamp max tilt target to 0.35 rad (~20 deg) to prevent aerodynamic stall and flip
        MAX_TILT = 0.35
        raw_pitch_target = (
            self.control_gains["velocity_xy_kp"] * np.clip(error_vx, -1.0, 1.0)
            + self.control_gains["velocity_xy_kd"] * np.clip(derivative_vx, -4.0, 4.0)
        )
        raw_roll_target = (
            -self.control_gains["velocity_xy_kp"] * np.clip(error_vy, -1.0, 1.0)
            - self.control_gains["velocity_xy_kd"] * np.clip(derivative_vy, -4.0, 4.0)
        )
        pitch_target = float(np.clip(raw_pitch_target, -MAX_TILT, MAX_TILT))
        roll_target  = float(np.clip(raw_roll_target,  -MAX_TILT, MAX_TILT))

        self.previous_velocity_x_error = error_vx
        self.previous_velocity_y_error = error_vy

        # ── Altitude PID (+48 hover thrust feedforward) ────────────────
        altitude_error = target_altitude - current_altitude
        altitude_derivative = (altitude_error - self.previous_altitude_error) / dt
        self.altitude_integral += altitude_error * dt
        altitude_output = (
            self.control_gains["altitude_kp"] * altitude_error
            + self.control_gains["altitude_kd"] * np.clip(altitude_derivative, -5.0, 5.0)
            + self.control_gains["altitude_ki"] * np.clip(self.altitude_integral, -2.0, 2.0)
            + 48.0
        )
        # Prevent altitude thrust saturation from dominating attitude control
        altitude_output = float(np.clip(altitude_output, 25.0, 75.0))
        self.previous_altitude_error = altitude_error

        # ── Attitude PID ──────────────────────────────────────────────
        error_pitch = pitch_target - current_pitch
        derivative_pitch = (error_pitch - self.previous_pitch_error) / dt
        error_roll = roll_target - current_roll
        derivative_roll = (error_roll - self.previous_roll_error) / dt

        roll_output = (
            self.control_gains["attitude_rp_kp"] * np.clip(error_roll, -1.0, 1.0)
            + self.control_gains["attitude_rp_kd"] * np.clip(derivative_roll, -10.0, 10.0)
        )
        pitch_output = (
            self.control_gains["attitude_rp_kp"] * np.clip(error_pitch, -1.0, 1.0)
            + self.control_gains["attitude_rp_kd"] * np.clip(derivative_pitch, -10.0, 10.0)
        )
        yaw_output = self.control_gains["attitude_yaw_kp"] * np.clip(
            target_yaw_rate - current_yaw_rate, -1.0, 1.0
        )

        self.previous_pitch_error = error_pitch
        self.previous_roll_error = error_roll

        # Store logged values
        self.logged_error_vx = error_vx
        self.logged_error_vy = error_vy
        self.logged_altitude_error = altitude_error
        self.logged_error_pitch = error_pitch
        self.logged_error_roll = error_roll
        self.logged_yaw_rate_error = target_yaw_rate - current_yaw_rate
        self.logged_altitude_output = altitude_output
        self.logged_pitch_target = pitch_target
        self.logged_roll_target = roll_target

        # ── Motor mixing (exact Bitcraze formula) ─────────────────────
        motor1 = float(np.clip(altitude_output - roll_output - pitch_output + yaw_output, 0.0, 600.0))
        motor2 = float(np.clip(altitude_output - roll_output + pitch_output - yaw_output, 0.0, 600.0))
        motor3 = float(np.clip(altitude_output + roll_output + pitch_output + yaw_output, 0.0, 600.0))
        motor4 = float(np.clip(altitude_output + roll_output - pitch_output - yaw_output, 0.0, 600.0))

        return [motor1, motor2, motor3, motor4]
