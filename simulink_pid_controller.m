%% ========================================================================
%  Simulink / MATLAB Code for Crazyflie PID Controller
%  Autonomous Drone Navigation — CS 666
%  ========================================================================
%  This script creates the PID controller model for the Crazyflie drone
%  used in the Autonomous Drone Navigation simulation.
%  
%  Controller Architecture:
%    Cascaded PID: Outer loop (velocity) → Inner loop (attitude) → Motor mixing
%  ========================================================================

clear; clc; close all;

%% ─────────────────────────────────────────────────────────────────────────
%  1. SYSTEM PARAMETERS
%  ─────────────────────────────────────────────────────────────────────────

% Crazyflie 2.1 Physical Parameters
drone.mass     = 0.027;       % kg
drone.g        = 9.81;        % m/s^2
drone.Ixx      = 1.43e-5;     % Roll inertia (kg·m²)
drone.Iyy      = 1.43e-5;     % Pitch inertia (kg·m²)
drone.Izz      = 2.89e-5;     % Yaw inertia (kg·m²)
drone.arm      = 0.046;       % Motor arm length (m)
drone.kf       = 3.16e-10;    % Thrust coefficient (N/rpm²)
drone.km       = 7.94e-12;    % Torque coefficient (N·m/rpm²)

% Simulation Parameters
sim.dt         = 0.008;       % Time step (s) — Webots default
sim.T          = 10;          % Total simulation time (s)
sim.t          = 0:sim.dt:sim.T;
sim.N          = length(sim.t);

%% ─────────────────────────────────────────────────────────────────────────
%  2. PID GAINS (from pid_controller.py)
%  ─────────────────────────────────────────────────────────────────────────

% Altitude PID (full PID with integral)
pid.altitude.Kp = 10;
pid.altitude.Ki = 5;
pid.altitude.Kd = 5;

% Velocity XY PD Controller
pid.velocity.Kp = 2;
pid.velocity.Kd = 0.5;

% Attitude Roll/Pitch PD Controller
pid.attitude.Kp = 0.5;
pid.attitude.Kd = 0.1;

% Yaw Rate P Controller
pid.yaw.Kp = 1;
pid.yaw.Kd = 0.5;

%% ─────────────────────────────────────────────────────────────────────────
%  3. SETPOINTS (Desired States)
%  ─────────────────────────────────────────────────────────────────────────

target.altitude  = 1.0;     % meters
target.vx        = 0.3;     % m/s (forward)
target.vy        = 0.0;     % m/s (sideways)
target.yaw_rate  = 0.0;     % rad/s

%% ─────────────────────────────────────────────────────────────────────────
%  4. STATE INITIALIZATION
%  ─────────────────────────────────────────────────────────────────────────

% Current states
state.altitude  = 0;
state.vz        = 0;
state.vx        = 0;
state.vy        = 0;
state.roll      = 0;
state.pitch     = 0;
state.yaw_rate  = 0;

% Error memory (for derivative computation)
err_prev.vx       = 0;
err_prev.vy       = 0;
err_prev.altitude = 0;
err_prev.pitch    = 0;
err_prev.roll     = 0;

% Integral accumulator
integral.altitude = 0;

% Logging arrays
log.altitude    = zeros(1, sim.N);
log.vx          = zeros(1, sim.N);
log.err_alt     = zeros(1, sim.N);
log.err_vx      = zeros(1, sim.N);
log.err_pitch   = zeros(1, sim.N);
log.err_roll    = zeros(1, sim.N);
log.motor       = zeros(4, sim.N);

%% ─────────────────────────────────────────────────────────────────────────
%  5. PID CONTROLLER SIMULATION LOOP
%  ─────────────────────────────────────────────────────────────────────────

fprintf('Running PID Controller Simulation...\n');

for k = 1:sim.N
    dt = sim.dt;
    
    %% ── 5.1: Velocity Error (Outer Loop) ──
    % X-velocity error and derivative
    err_vx = target.vx - state.vx;
    derr_vx = (err_vx - err_prev.vx) / dt;
    
    % Y-velocity error and derivative
    err_vy = target.vy - state.vy;
    derr_vy = (err_vy - err_prev.vy) / dt;
    
    % Compute desired pitch and roll from velocity errors
    pitch_target = pid.velocity.Kp * max(min(err_vx, 1), -1) + ...
                   pid.velocity.Kd * derr_vx;
    roll_target  = -(pid.velocity.Kp * max(min(err_vy, 1), -1) + ...
                     pid.velocity.Kd * derr_vy);
    
    % Store previous errors
    err_prev.vx = err_vx;
    err_prev.vy = err_vy;
    
    %% ── 5.2: Altitude PID (with Integral) ──
    err_alt = target.altitude - state.altitude;
    derr_alt = (err_alt - err_prev.altitude) / dt;
    integral.altitude = integral.altitude + err_alt * dt;
    integral.altitude = max(min(integral.altitude, 2), -2);  % Anti-windup
    
    alt_output = pid.altitude.Kp * err_alt + ...
                 pid.altitude.Kd * derr_alt + ...
                 pid.altitude.Ki * integral.altitude + 48;  % Hover offset
    
    err_prev.altitude = err_alt;
    
    %% ── 5.3: Attitude PID (Inner Loop) ──
    err_pitch = pitch_target - state.pitch;
    derr_pitch = (err_pitch - err_prev.pitch) / dt;
    
    err_roll = roll_target - state.roll;
    derr_roll = (err_roll - err_prev.roll) / dt;
    
    roll_output  = pid.attitude.Kp * max(min(err_roll, 1), -1) + ...
                   pid.attitude.Kd * derr_roll;
    pitch_output = pid.attitude.Kp * max(min(err_pitch, 1), -1) + ...
                   pid.attitude.Kd * derr_pitch;
    
    % Yaw rate control
    yaw_output = pid.yaw.Kp * max(min(target.yaw_rate - state.yaw_rate, 1), -1);
    
    err_prev.pitch = err_pitch;
    err_prev.roll  = err_roll;
    
    %% ── 5.4: Motor Mixing ──
    % Motor commands (quadrotor X-configuration)
    %   M1 = altitude - roll - pitch + yaw  (front-left)
    %   M2 = altitude - roll + pitch - yaw  (front-right)
    %   M3 = altitude + roll + pitch + yaw  (rear-right)
    %   M4 = altitude + roll - pitch - yaw  (rear-left)
    
    motor(1) = max(min(alt_output - roll_output - pitch_output + yaw_output, 600), 0);
    motor(2) = max(min(alt_output - roll_output + pitch_output - yaw_output, 600), 0);
    motor(3) = max(min(alt_output + roll_output + pitch_output + yaw_output, 600), 0);
    motor(4) = max(min(alt_output + roll_output - pitch_output - yaw_output, 600), 0);
    
    %% ── 5.5: Simplified Plant Dynamics ──
    % (In Webots, this is handled by the physics engine)
    % Here we use first-order approximations for visualization
    
    tau_alt = 0.8;    % Altitude time constant
    tau_vel = 0.3;    % Velocity time constant
    tau_att = 0.1;    % Attitude time constant
    
    state.altitude = state.altitude + (target.altitude - state.altitude) / tau_alt * dt * ...
                     (alt_output / 48);  % Normalized by hover thrust
    state.vx = state.vx + (err_vx * 0.5) * dt;
    state.pitch = state.pitch + (pitch_target - state.pitch) / tau_att * dt;
    state.roll  = state.roll  + (roll_target  - state.roll)  / tau_att * dt;
    
    %% ── 5.6: Log Data ──
    log.altitude(k)  = state.altitude;
    log.vx(k)        = state.vx;
    log.err_alt(k)   = err_alt;
    log.err_vx(k)    = err_vx;
    log.err_pitch(k) = err_pitch;
    log.err_roll(k)  = err_roll;
    log.motor(:, k)  = motor';
end

fprintf('Simulation complete.\n\n');

%% ─────────────────────────────────────────────────────────────────────────
%  6. PLOT RESULTS
%  ─────────────────────────────────────────────────────────────────────────

figure('Position', [100 100 1400 900], 'Color', 'w');
sgtitle('PID Controller Response — Crazyflie Drone (CS 666)', ...
        'FontSize', 18, 'FontWeight', 'bold');

% Altitude Error
subplot(3, 2, 1);
plot(sim.t, log.err_alt, 'r-', 'LineWidth', 1.8);
hold on; yline(0, '--', 'Color', [0.5 0.5 0.5]); hold off;
title('Altitude Error (Kp=10, Ki=5, Kd=5)', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Error (m)');
grid on; set(gca, 'GridAlpha', 0.3);

% Altitude Response
subplot(3, 2, 2);
plot(sim.t, log.altitude, 'r-', 'LineWidth', 1.8); hold on;
yline(target.altitude, 'k--', 'LineWidth', 1.2); hold off;
title('Altitude Step Response', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Altitude (m)');
legend('Actual', 'Setpoint', 'Location', 'southeast');
grid on; set(gca, 'GridAlpha', 0.3);

% Velocity X Error
subplot(3, 2, 3);
plot(sim.t, log.err_vx, 'b-', 'LineWidth', 1.8);
hold on; yline(0, '--', 'Color', [0.5 0.5 0.5]); hold off;
title('Forward Velocity Error (Kp=2, Kd=0.5)', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Error (m/s)');
grid on; set(gca, 'GridAlpha', 0.3);

% Pitch Error
subplot(3, 2, 4);
plot(sim.t, log.err_pitch, 'Color', [0.95 0.61 0.07], 'LineWidth', 1.8);
hold on; yline(0, '--', 'Color', [0.5 0.5 0.5]); hold off;
title('Pitch Error (Kp=0.5, Kd=0.1)', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Error (rad)');
grid on; set(gca, 'GridAlpha', 0.3);

% Roll Error
subplot(3, 2, 5);
plot(sim.t, log.err_roll, 'Color', [0.61 0.35 0.71], 'LineWidth', 1.8);
hold on; yline(0, '--', 'Color', [0.5 0.5 0.5]); hold off;
title('Roll Error (Kp=0.5, Kd=0.1)', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Error (rad)');
grid on; set(gca, 'GridAlpha', 0.3);

% Motor Outputs
subplot(3, 2, 6);
plot(sim.t, log.motor(1,:), 'LineWidth', 1.2); hold on;
plot(sim.t, log.motor(2,:), 'LineWidth', 1.2);
plot(sim.t, log.motor(3,:), 'LineWidth', 1.2);
plot(sim.t, log.motor(4,:), 'LineWidth', 1.2); hold off;
title('Motor Commands', 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Motor Speed (cmd)');
legend('M1', 'M2', 'M3', 'M4', 'Location', 'northeast');
grid on; set(gca, 'GridAlpha', 0.3);

% Save figure
saveas(gcf, 'pid_controller_simulink_response.png');
fprintf('Figure saved: pid_controller_simulink_response.png\n');

%% ─────────────────────────────────────────────────────────────────────────
%  7. SIMULINK MODEL GENERATION (Programmatic)
%  ─────────────────────────────────────────────────────────────────────────
%
%  To create the Simulink model, run: create_simulink_model()
%  This requires MATLAB R2020b+ with Simulink installed.
%
%  The model structure is:
%
%  ┌─────────────┐     ┌───────────────┐     ┌──────────────┐     ┌──────────┐
%  │  Setpoints   │────▶│ Velocity PID  │────▶│ Attitude PID │────▶│  Motor   │
%  │ (alt, vx,vy) │     │ (Kp=2,Kd=0.5)│     │(Kp=0.5,Kd=0.1)   │  Mixer   │
%  └─────────────┘     └───────────────┘     └──────────────┘     └──────────┘
%                                                                       │
%  ┌─────────────┐     ┌───────────────┐                                │
%  │  alt target  │────▶│ Altitude PID  │───────────────────────────────▶│
%  │   (1.0 m)   │     │(Kp=10,Ki=5,   │                               │
%  └─────────────┘     │ Kd=5)         │                          ┌─────▼─────┐
%                      └───────────────┘                          │  Crazyflie │
%                                                                 │   Plant    │
%                                                                 └───────────┘

function create_simulink_model()
    % Creates a Simulink model for the Crazyflie PID controller
    % Run this function in MATLAB with Simulink installed
    
    model_name = 'CrazyfliePIDController';
    
    % Create new model
    new_system(model_name);
    open_system(model_name);
    
    % Add blocks
    % --- Setpoints ---
    add_block('simulink/Sources/Constant', [model_name '/Alt_Setpoint'], ...
        'Value', '1.0', 'Position', [50 50 100 70]);
    add_block('simulink/Sources/Constant', [model_name '/Vx_Setpoint'], ...
        'Value', '0.3', 'Position', [50 120 100 140]);
    add_block('simulink/Sources/Constant', [model_name '/Vy_Setpoint'], ...
        'Value', '0.0', 'Position', [50 190 100 210]);
    
    % --- Sum blocks (error computation) ---
    add_block('simulink/Math Operations/Sum', [model_name '/Alt_Error'], ...
        'Inputs', '+-', 'Position', [180 50 210 80]);
    add_block('simulink/Math Operations/Sum', [model_name '/Vx_Error'], ...
        'Inputs', '+-', 'Position', [180 120 210 150]);
    add_block('simulink/Math Operations/Sum', [model_name '/Vy_Error'], ...
        'Inputs', '+-', 'Position', [180 190 210 220]);
    
    % --- PID Controllers ---
    add_block('simulink/Continuous/PID Controller', [model_name '/Altitude_PID'], ...
        'P', '10', 'I', '5', 'D', '5', 'Position', [280 40 380 90]);
    add_block('simulink/Continuous/PID Controller', [model_name '/Velocity_X_PD'], ...
        'P', '2', 'I', '0', 'D', '0.5', 'Position', [280 110 380 160]);
    add_block('simulink/Continuous/PID Controller', [model_name '/Velocity_Y_PD'], ...
        'P', '2', 'I', '0', 'D', '0.5', 'Position', [280 180 380 230]);
    
    % --- Attitude PID ---
    add_block('simulink/Continuous/PID Controller', [model_name '/Pitch_PD'], ...
        'P', '0.5', 'I', '0', 'D', '0.1', 'Position', [500 110 600 160]);
    add_block('simulink/Continuous/PID Controller', [model_name '/Roll_PD'], ...
        'P', '0.5', 'I', '0', 'D', '0.1', 'Position', [500 180 600 230]);
    
    % --- Motor Mixer (MATLAB Function) ---
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model_name '/Motor_Mixer'], 'Position', [700 100 820 200]);
    
    % --- Plant (Transfer Function) ---
    add_block('simulink/Continuous/Transfer Fcn', [model_name '/Plant'], ...
        'Numerator', '[1]', 'Denominator', '[0.8 1]', ...
        'Position', [900 130 1000 170]);
    
    % --- Scopes ---
    add_block('simulink/Sinks/Scope', [model_name '/Error_Scope'], ...
        'Position', [1100 50 1150 100]);
    add_block('simulink/Sinks/Scope', [model_name '/Motor_Scope'], ...
        'Position', [1100 130 1150 180]);
    
    % Connect blocks
    add_line(model_name, 'Alt_Setpoint/1', 'Alt_Error/1');
    add_line(model_name, 'Alt_Error/1', 'Altitude_PID/1');
    add_line(model_name, 'Vx_Setpoint/1', 'Vx_Error/1');
    add_line(model_name, 'Vx_Error/1', 'Velocity_X_PD/1');
    
    % Save and close
    save_system(model_name);
    fprintf('Simulink model created: %s.slx\n', model_name);
end

%% ─────────────────────────────────────────────────────────────────────────
%  8. MOTOR MIXER FUNCTION (for Simulink MATLAB Function block)
%  ─────────────────────────────────────────────────────────────────────────

function [m1, m2, m3, m4] = motor_mixer(alt_cmd, roll_cmd, pitch_cmd, yaw_cmd)
    % MOTOR_MIXER Compute individual motor commands from control outputs
    %
    %   Quadrotor X-configuration motor mixing:
    %   M1 (front-left)  = altitude - roll - pitch + yaw
    %   M2 (front-right) = altitude - roll + pitch - yaw
    %   M3 (rear-right)  = altitude + roll + pitch + yaw
    %   M4 (rear-left)   = altitude + roll - pitch - yaw
    
    m1 = max(min(alt_cmd - roll_cmd - pitch_cmd + yaw_cmd, 600), 0);
    m2 = max(min(alt_cmd - roll_cmd + pitch_cmd - yaw_cmd, 600), 0);
    m3 = max(min(alt_cmd + roll_cmd + pitch_cmd + yaw_cmd, 600), 0);
    m4 = max(min(alt_cmd + roll_cmd - pitch_cmd - yaw_cmd, 600), 0);
end

%% ─────────────────────────────────────────────────────────────────────────
%  9. DISPLAY SUMMARY
%  ─────────────────────────────────────────────────────────────────────────

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║    Crazyflie PID Controller — MATLAB/Simulink Code     ║\n');
fprintf('║    Autonomous Drone Navigation — CS 666                ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Controller Architecture:                              ║\n');
fprintf('║  ┌────────────┐  ┌────────────┐  ┌──────────┐         ║\n');
fprintf('║  │ Velocity   │─▶│ Attitude   │─▶│  Motor   │         ║\n');
fprintf('║  │ PID (outer)│  │ PID (inner)│  │  Mixer   │         ║\n');
fprintf('║  └────────────┘  └────────────┘  └──────────┘         ║\n');
fprintf('║                                                        ║\n');
fprintf('║  Altitude PID:  Kp=10, Ki=5, Kd=5                     ║\n');
fprintf('║  Velocity PD:   Kp=2,  Kd=0.5                         ║\n');
fprintf('║  Attitude PD:   Kp=0.5, Kd=0.1                        ║\n');
fprintf('║  Yaw P:         Kp=1,  Kd=0.5                         ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
