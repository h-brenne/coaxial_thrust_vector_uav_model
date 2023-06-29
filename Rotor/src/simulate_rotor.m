clear
% Setup aerodynamics model
BEMT_config = initBEMTconfig('../Rotor_Configs/40DegRotor/BEMTconfig.m');
% Setup rotor dynamic model
rotor = importrobot('../Rotor_Configs/40DegRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
rotor.Gravity= [0 0 -9.81];
showdetails(rotor)
% Set parameter needed for aerodynamics
transform_positive_lag_pitch = getTransform(rotor, homeConfiguration(rotor), 'hub_1', 'side_hub_positive_1');
BEMT_config.lag_pitch_hinge_offset = transform_positive_lag_pitch(1,4);

numJoints = numel(homeConfiguration(rotor));

% Velocities given in hz to match motor driver
min_omega = 80.0*2*pi;
max_omega = 80.0*2*pi;
step_omega = 5.0*2*pi;
min_amplitude = 0.0;
max_amplitude = 0.3;
step_amplitude = 0.26;
min_phase = 0.0;
max_phase = 0.0;
step_phase = 1.0;
step_length_seconds = 0.4;

% This is mostly used to limit the real test system, although it might give
% steps that converge faster
enable_dual_amplitude_steps = false;

[omegas, amplitudes, phases] = generateThrustVectorSequence(min_omega,...
    max_omega, step_omega, min_amplitude, max_amplitude, step_amplitude,...
    min_phase, max_phase, step_phase, enable_dual_amplitude_steps);


experiment_length_seconds = step_length_seconds * length(omegas);


% Set up simulation parameters
tspan = [0 experiment_length_seconds];
q0 = zeros(numJoints,1);
qd0 = zeros(numJoints,1);
qd0(1) = omegas(1);
% Add some integrated speed error to start with
e = 2.0;
x0 = [q0; qd0; e];

[t,y] = ode45(@(t,y) odefcn(t,y,rotor, BEMT_config, omegas, ...
    amplitudes, phases, step_length_seconds), tspan, x0);
% Can't(easily) save non state data during ode solver, recreate them based on
% states
% This doubles the sim time for stahlhut solver, consider saving data
% during ode solver
skip_steps = 1;
[OpRot_positive, OpRot_negative, motor_torque, omega, amplitude, phase, omega_tilde] = recreate_data(BEMT_config,...
    rotor, y, t, omegas, amplitudes, phases, step_length_seconds, skip_steps);

save('../Results/step026A_80hz_leishman_new.mat', 't','y', 'rotor', 'BEMT_config', 'OpRot_negative', 'OpRot_positive', ...
    'motor_torque', 'omega', 'amplitude', 'phase', 'omega_tilde');




function [external_torques, f_ext]  = get_external_forces_and_torques(BEMT_config,...
    rotor, q, qd, e, omega, speed_setpoint)
    % External moments consists of aerodynamic forces and motor torque
    % Omega is the average rotor speed control input
    % This is used for momentum theory, while BET blade velocities
    % are calculated from (q,qd)

    motor_torque = pi_motor_controller(qd(1), e, speed_setpoint);
    f_ext = getAerodynamicForcesAndMoments(BEMT_config,rotor, q, qd, omega);
    external_torques = [motor_torque;0;0;0];
end

function x_dot = odefcn(t,x, rotor, BEMT_config, omegas, amplitudes,...
    phases, command_step_length)
% 4 generalized variables. 
% rotor position
% teetering hinge
% positive lag-pitch hinge
% negative lag-pitch hinge
    disp(t)
    x_dot = zeros(8,1);
    q = x(1:4);
    qd = x(5:8);
    % Motor speed error integral
    e = x(9);

    % Compute the current command step index
    if t == 0
        current_step = 1;
    else
        current_step = ceil((t) / command_step_length);
    end
    
    omega = omegas(current_step);
    omega_tilde = sinusoidal_control_law(q(1), omega,...
        amplitudes(current_step), phases(current_step));
    speed_setpoint = (omega + omega_tilde);
    e_dot = speed_setpoint - qd(1); 

    [external_torques, f_ext] = get_external_forces_and_torques(BEMT_config,...
        rotor, q, qd, e, omega, speed_setpoint);
    qdd = forwardDynamics(rotor, q, qd, external_torques, f_ext);
    x_dot(1:4) = qd;
    x_dot(5:8) = qdd;
    x_dot(9) = e_dot;
end