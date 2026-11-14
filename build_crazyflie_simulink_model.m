%% ========================================================================
%  CRAZYFLIE QUADROTOR CASCADED PID CONTROLLER — SIMULINK MODEL BUILDER
%  Autonomous Drone Navigation (CS 666)
%  ========================================================================
%  This script programmatically generates, wires, and opens a complete
%  Simulink block diagram (.slx) for the cascaded dual-loop PID controller.
%
%  Features:
%    - Outer Loop: Altitude PID (Anti-windup + Bias) & Velocity XY PD -> Attitude targets
%    - Inner Loop: Attitude PD (Pitch & Roll) & Yaw Rate P
%    - Motor Mixer Subsystem: Quadrotor X-configuration with [0, 600] saturation
%    - Closed-loop Plant Dynamics (First-order drone response approximations)
%    - Scopes for Setpoints, Attitude, Altitude, and Motor Speeds
%
%  To Run:
%    >> build_crazyflie_simulink_model
%  ========================================================================

clear; clc; close all;

model_name = 'Crazyflie_Cascaded_PID';

fprintf('==> Initializing Simulink model: %s.slx\n', model_name);

% Close existing model if open (without saving discard changes)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new system and open GUI editor
new_system(model_name);
open_system(model_name);

% Set simulation parameters (Fixed step 0.008s matching Webots physics)
set_param(model_name, 'SolverType', 'Fixed-step', ...
                      'Solver', 'ode4', ...
                      'FixedStep', '0.008', ...
                      'StopTime', '10.0');

%% ========================================================================
%  STAGE 1: REFERENCE SETPOINTS (SOURCES)
%  ========================================================================
fprintf('--> Creating Reference Setpoints...\n');

% Altitude Step: 0m -> 1.0m at t=0.5s
add_block('simulink/Sources/Step', [model_name '/Alt_Setpoint'], ...
    'Time', '0.5', 'Before', '0', 'After', '1.0', ...
    'Position', [40, 60, 90, 90]);

% Velocity X Step: 0 -> 0.3 m/s at t=1.0s
add_block('simulink/Sources/Step', [model_name '/Vx_Setpoint'], ...
    'Time', '1.0', 'Before', '0', 'After', '0.3', ...
    'Position', [40, 180, 90, 210]);

% Velocity Y Step: constant 0 m/s
add_block('simulink/Sources/Constant', [model_name '/Vy_Setpoint'], ...
    'Value', '0.0', ...
    'Position', [40, 300, 90, 330]);

% Desired Yaw Rate: constant 0 rad/s
add_block('simulink/Sources/Constant', [model_name '/YawRate_Setpoint'], ...
    'Value', '0.0', ...
    'Position', [40, 420, 90, 450]);

%% ========================================================================
%  STAGE 2: ERROR SUM BLOCKS (COMPARATORS)
%  ========================================================================
fprintf('--> Creating Error Comparators...\n');

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Alt_Error'], ...
    'Inputs', '+-', 'Position', [150, 65, 175, 95]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Vx_Error'], ...
    'Inputs', '+-', 'Position', [150, 185, 175, 215]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Vy_Error'], ...
    'Inputs', '+-', 'Position', [150, 305, 175, 335]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Yaw_Error'], ...
    'Inputs', '+-', 'Position', [150, 425, 175, 455]);

%% ========================================================================
%  STAGE 3: ERROR SATURATIONS ([-1, 1] clipping as in pid_controller.py)
%  ========================================================================
fprintf('--> Adding Error Saturation Limiters [-1, 1]...\n');

add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Vx_Err'], ...
    'UpperLimit', '1.0', 'LowerLimit', '-1.0', ...
    'Position', [220, 185, 250, 215]);

add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Vy_Err'], ...
    'UpperLimit', '1.0', 'LowerLimit', '-1.0', ...
    'Position', [220, 305, 250, 335]);

add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Yaw_Err'], ...
    'UpperLimit', '1.0', 'LowerLimit', '-1.0', ...
    'Position', [220, 425, 250, 455]);

%% ========================================================================
%  STAGE 4: OUTER LOOP CONTROLLERS (ALTITUDE & VELOCITY)
%  ========================================================================
fprintf('--> Creating Outer-Loop Controllers...\n');

% Altitude PID: Kp=10, Ki=5, Kd=5 with anti-windup clamping [-2, 2]
add_block('simulink/Continuous/PID Controller', [model_name '/Altitude_PID'], ...
    'Controller', 'PID', ...
    'P', '10', 'I', '5', 'D', '5', 'FilterGain', '100', ...
    'AntiWindupMode', 'clamping', ...
    'Position', [280, 55, 360, 105]);

% Altitude Hover Thrust Bias (+48)
add_block('simulink/Math Operations/Bias', [model_name '/Hover_Bias'], ...
    'Bias', '48', ...
    'Position', [400, 65, 440, 95]);

% Velocity X PD (Kp=2, Kd=0.5) -> Outputs Pitch Target
add_block('simulink/Continuous/PID Controller', [model_name '/Vel_X_PD'], ...
    'Controller', 'PD', ...
    'P', '2', 'D', '0.5', 'FilterGain', '100', ...
    'Position', [290, 175, 370, 225]);

% Velocity Y PD (Kp=2, Kd=0.5)
add_block('simulink/Continuous/PID Controller', [model_name '/Vel_Y_PD'], ...
    'Controller', 'PD', ...
    'P', '2', 'D', '0.5', 'FilterGain', '100', ...
    'Position', [290, 295, 370, 345]);

% Invert Vy output for Roll Target: roll_target = - (Kp*err_vy + Kd*derr_vy)
add_block('simulink/Math Operations/Gain', [model_name '/Invert_Vy_Gain'], ...
    'Gain', '-1', ...
    'Position', [400, 305, 435, 335]);

%% ========================================================================
%  STAGE 5: INNER LOOP ATTITUDE SUM BLOCKS
%  ========================================================================
fprintf('--> Creating Inner-Loop Comparators...\n');

% Pitch Error = Desired Pitch - Actual Pitch
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pitch_Error'], ...
    'Inputs', '+-', 'Position', [490, 185, 515, 215]);

% Roll Error = Desired Roll - Actual Roll
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Roll_Error'], ...
    'Inputs', '+-', 'Position', [490, 305, 515, 335]);

%% ========================================================================
%  STAGE 6: INNER LOOP CONTROLLERS (PITCH, ROLL, YAW)
%  ========================================================================
fprintf('--> Creating Inner-Loop Attitude Controllers...\n');

% Pitch PD: Kp=0.5, Kd=0.1
add_block('simulink/Continuous/PID Controller', [model_name '/Pitch_PD'], ...
    'Controller', 'PD', ...
    'P', '0.5', 'D', '0.1', 'FilterGain', '100', ...
    'Position', [560, 175, 640, 225]);

% Roll PD: Kp=0.5, Kd=0.1
add_block('simulink/Continuous/PID Controller', [model_name '/Roll_PD'], ...
    'Controller', 'PD', ...
    'P', '0.5', 'D', '0.1', 'FilterGain', '100', ...
    'Position', [560, 295, 640, 345]);

% Yaw Rate P Controller: Kp=1.0
add_block('simulink/Math Operations/Gain', [model_name '/Yaw_Rate_P'], ...
    'Gain', '1.0', ...
    'Position', [320, 425, 360, 455]);

%% ========================================================================
%  STAGE 7: MOTOR MIXER SUBSYSTEM (Quadrotor X-Configuration)
%  ========================================================================
fprintf('--> Building Motor Mixer Subsystem...\n');

mixer_sys = [model_name '/Motor_Mixer'];
add_block('simulink/Commonly Used Blocks/Subsystem', mixer_sys, ...
    'Position', [720, 120, 840, 400]);

% Clean default Subsystem contents
delete_line(mixer_sys, 'In1/1', 'Out1/1');
delete_block([mixer_sys '/In1']);
delete_block([mixer_sys '/Out1']);

% Mixer Inputs: 1:Alt_Cmd, 2:Pitch_Cmd, 3:Roll_Cmd, 4:Yaw_Cmd
add_block('simulink/Sources/Inport', [mixer_sys '/Alt_Cmd'], ...
    'Port', '1', 'Position', [30, 40, 60, 55]);
add_block('simulink/Sources/Inport', [mixer_sys '/Pitch_Cmd'], ...
    'Port', '2', 'Position', [30, 120, 60, 135]);
add_block('simulink/Sources/Inport', [mixer_sys '/Roll_Cmd'], ...
    'Port', '3', 'Position', [30, 200, 60, 215]);
add_block('simulink/Sources/Inport', [mixer_sys '/Yaw_Cmd'], ...
    'Port', '4', 'Position', [30, 280, 60, 295]);

% Equations:
% M1 (Front-Left)  = Alt - Roll - Pitch + Yaw
% M2 (Front-Right) = Alt - Roll + Pitch - Yaw
% M3 (Rear-Right)  = Alt + Roll + Pitch + Yaw
% M4 (Rear-Left)   = Alt + Roll - Pitch - Yaw
signs = {'+--+', '+-+-', '++++', '++--'};
motor_names = {'M1_FrontLeft', 'M2_FrontRight', 'M3_RearRight', 'M4_RearLeft'};

for i = 1:4
    y_pos = 40 + (i-1)*90;
    sum_blk = sprintf('%s/Sum_M%d', mixer_sys, i);
    sat_blk = sprintf('%s/Sat_M%d', mixer_sys, i);
    out_blk = sprintf('%s/%s', mixer_sys, motor_names{i});
    
    add_block('simulink/Math Operations/Sum', sum_blk, ...
        'Inputs', signs{i}, 'Position', [180, y_pos, 210, y_pos+40]);
    
    add_block('simulink/Discontinuities/Saturation', sat_blk, ...
        'UpperLimit', '600', 'LowerLimit', '0', ...
        'Position', [260, y_pos+5, 300, y_pos+35]);
        
    add_block('simulink/Sinks/Outport', out_blk, ...
        'Port', num2str(i), 'Position', [350, y_pos+10, 380, y_pos+30]);
        
    add_line(mixer_sys, sprintf('Sum_M%d/1', i), sprintf('Sat_M%d/1', i));
    add_line(mixer_sys, sprintf('Sat_M%d/1', i), sprintf('%s/1', motor_names{i}));
    
    % Wire inputs to each motor sum block
    add_line(mixer_sys, 'Alt_Cmd/1', sprintf('Sum_M%d/1', i), 'autorouting', 'on');
    add_line(mixer_sys, 'Roll_Cmd/1', sprintf('Sum_M%d/2', i), 'autorouting', 'on');
    add_line(mixer_sys, 'Pitch_Cmd/1', sprintf('Sum_M%d/3', i), 'autorouting', 'on');
    add_line(mixer_sys, 'Yaw_Cmd/1', sprintf('Sum_M%d/4', i), 'autorouting', 'on');
end

%% ========================================================================
%  STAGE 8: CRAZYFLIE QUADROTOR PLANT DYNAMICS (Continuous Approximations)
%  ========================================================================
fprintf('--> Creating Plant Dynamics Blocks...\n');

% Altitude dynamics: G_alt(s) = 1 / (0.8s + 1)
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Altitude_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.8 1]', ...
    'Position', [910, 65, 1000, 105]);

% Pitch dynamics: G_pitch(s) = 1 / (0.1s + 1)
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Pitch_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.1 1]', ...
    'Position', [910, 185, 1000, 225]);

% Roll dynamics: G_roll(s) = 1 / (0.1s + 1)
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Roll_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.1 1]', ...
    'Position', [910, 305, 1000, 345]);

% Velocity dynamics: derived from pitch/roll tilt
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Vx_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.3 1]', ...
    'Position', [1040, 185, 1110, 225]);

add_block('simulink/Continuous/Transfer Fcn', [model_name '/Vy_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.3 1]', ...
    'Position', [1040, 305, 1110, 345]);

% Yaw rate dynamics
add_block('simulink/Continuous/Transfer Fcn', [model_name '/YawRate_Plant'], ...
    'Numerator', '[1]', 'Denominator', '[0.05 1]', ...
    'Position', [910, 425, 1000, 465]);

%% ========================================================================
%  STAGE 9: SINKS & SCOPES
%  ========================================================================
fprintf('--> Adding Monitoring Scopes...\n');

% Altitude Scope: Port 1 = Ref, Port 2 = Actual
add_block('simulink/Sinks/Scope', [model_name '/Altitude_Scope'], ...
    'NumInputPorts', '2', 'Position', [1180, 50, 1220, 100]);

% Velocity XY Scope: Port 1 = Vx, Port 2 = Vy
add_block('simulink/Sinks/Scope', [model_name '/Velocity_Scope'], ...
    'NumInputPorts', '2', 'Position', [1180, 230, 1220, 280]);

% Attitude Scope: Port 1 = Pitch, Port 2 = Roll
add_block('simulink/Sinks/Scope', [model_name '/Attitude_Scope'], ...
    'NumInputPorts', '2', 'Position', [1180, 330, 1220, 380]);

% Motor Command Scope: 4 Channels (M1, M2, M3, M4)
add_block('simulink/Sinks/Scope', [model_name '/Motor_Commands_Scope'], ...
    'NumInputPorts', '4', 'Position', [910, 510, 950, 610]);

%% ========================================================================
%  STAGE 10: WIRING ALL SIGNALS AND FEEDBACK LOOPS
%  ========================================================================
fprintf('--> Connecting All Signals and Feedback Loops...\n');

% --- Altitude Loop ---
add_line(model_name, 'Alt_Setpoint/1', 'Sum_Alt_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Alt_Error/1', 'Altitude_PID/1', 'autorouting', 'on');
add_line(model_name, 'Altitude_PID/1', 'Hover_Bias/1', 'autorouting', 'on');
add_line(model_name, 'Hover_Bias/1', 'Motor_Mixer/1', 'autorouting', 'on');
add_line(model_name, 'Hover_Bias/1', 'Altitude_Plant/1', 'autorouting', 'on');
add_line(model_name, 'Altitude_Plant/1', 'Altitude_Scope/2', 'autorouting', 'on');
add_line(model_name, 'Altitude_Plant/1', 'Sum_Alt_Error/2', 'autorouting', 'on'); % Feedback
add_line(model_name, 'Alt_Setpoint/1', 'Altitude_Scope/1', 'autorouting', 'on');

% --- Velocity X & Pitch Loop ---
add_line(model_name, 'Vx_Setpoint/1', 'Sum_Vx_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Vx_Error/1', 'Sat_Vx_Err/1', 'autorouting', 'on');
add_line(model_name, 'Sat_Vx_Err/1', 'Vel_X_PD/1', 'autorouting', 'on');
add_line(model_name, 'Vel_X_PD/1', 'Sum_Pitch_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Pitch_Error/1', 'Pitch_PD/1', 'autorouting', 'on');
add_line(model_name, 'Pitch_PD/1', 'Motor_Mixer/2', 'autorouting', 'on');
add_line(model_name, 'Pitch_PD/1', 'Pitch_Plant/1', 'autorouting', 'on');
add_line(model_name, 'Pitch_Plant/1', 'Vx_Plant/1', 'autorouting', 'on');
add_line(model_name, 'Pitch_Plant/1', 'Attitude_Scope/1', 'autorouting', 'on');
add_line(model_name, 'Pitch_Plant/1', 'Sum_Pitch_Error/2', 'autorouting', 'on'); % Pitch feedback
add_line(model_name, 'Vx_Plant/1', 'Velocity_Scope/1', 'autorouting', 'on');
add_line(model_name, 'Vx_Plant/1', 'Sum_Vx_Error/2', 'autorouting', 'on'); % Vx feedback

% --- Velocity Y & Roll Loop ---
add_line(model_name, 'Vy_Setpoint/1', 'Sum_Vy_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Vy_Error/1', 'Sat_Vy_Err/1', 'autorouting', 'on');
add_line(model_name, 'Sat_Vy_Err/1', 'Vel_Y_PD/1', 'autorouting', 'on');
add_line(model_name, 'Vel_Y_PD/1', 'Invert_Vy_Gain/1', 'autorouting', 'on');
add_line(model_name, 'Invert_Vy_Gain/1', 'Sum_Roll_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Roll_Error/1', 'Roll_PD/1', 'autorouting', 'on');
add_line(model_name, 'Roll_PD/1', 'Motor_Mixer/3', 'autorouting', 'on');
add_line(model_name, 'Roll_PD/1', 'Roll_Plant/1', 'autorouting', 'on');
add_line(model_name, 'Roll_Plant/1', 'Vy_Plant/1', 'autorouting', 'on');
add_line(model_name, 'Roll_Plant/1', 'Attitude_Scope/2', 'autorouting', 'on');
add_line(model_name, 'Roll_Plant/1', 'Sum_Roll_Error/2', 'autorouting', 'on'); % Roll feedback
add_line(model_name, 'Vy_Plant/1', 'Velocity_Scope/2', 'autorouting', 'on');
add_line(model_name, 'Vy_Plant/1', 'Sum_Vy_Error/2', 'autorouting', 'on'); % Vy feedback

% --- Yaw Loop ---
add_line(model_name, 'YawRate_Setpoint/1', 'Sum_Yaw_Error/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Yaw_Error/1', 'Sat_Yaw_Err/1', 'autorouting', 'on');
add_line(model_name, 'Sat_Yaw_Err/1', 'Yaw_Rate_P/1', 'autorouting', 'on');
add_line(model_name, 'Yaw_Rate_P/1', 'Motor_Mixer/4', 'autorouting', 'on');
add_line(model_name, 'Yaw_Rate_P/1', 'YawRate_Plant/1', 'autorouting', 'on');
add_line(model_name, 'YawRate_Plant/1', 'Sum_Yaw_Error/2', 'autorouting', 'on'); % Yaw feedback

% --- Motor Commands to Scope ---
add_line(model_name, 'Motor_Mixer/1', 'Motor_Commands_Scope/1', 'autorouting', 'on');
add_line(model_name, 'Motor_Mixer/2', 'Motor_Commands_Scope/2', 'autorouting', 'on');
add_line(model_name, 'Motor_Mixer/3', 'Motor_Commands_Scope/3', 'autorouting', 'on');
add_line(model_name, 'Motor_Mixer/4', 'Motor_Commands_Scope/4', 'autorouting', 'on');

%% ========================================================================
%  STAGE 11: SAVE AND FINALIZE
%  ========================================================================
save_system(model_name);

fprintf('\n');
fprintf('=======================================================================\n');
fprintf('  SUCCESS: Simulink Block Diagram "%s.slx" created!\n', model_name);
fprintf('  - Open in Simulink window to view complete graphical layout.\n');
fprintf('  - Click "Run" (Green Play button) or execute: sim(''%s'')\n', model_name);
fprintf('=======================================================================\n\n');
