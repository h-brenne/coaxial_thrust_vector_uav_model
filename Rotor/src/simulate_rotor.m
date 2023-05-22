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

% Set up simulation parameters
tspan = [0 0.6];
q0 = zeros(numJoints,1);
qd0 = zeros(numJoints,1);
qd0(1) = convangvel(4000, 'rpm', 'rad/s');
x0 = [q0; qd0];

[t,y] = ode45(@(t,y) odefcn(t,y,rotor, BEMT_config), tspan, x0);

save('../Results/test.mat', 't','y', 'rotor', 'BEMT_config');

function motor_torque = motor_controller(q, qd, t)
    % P controller for now
    Kp = 0.003;
    
    velocity_setpoint = convangvel(4000, 'rpm', 'rad/s');
    amplitude = 0.3;
    phase= 0;
    if t>0.2
        phase = pi;
    end
    if t> 0.4
        phase = pi/2;
    end
    velocity_setpoint_perturbed = velocity_setpoint*(1 + amplitude*sin(q(1)+phase));
    velocity_error = velocity_setpoint_perturbed - qd(1);
    motor_torque = Kp*velocity_error;
end


function [external_torques, f_ext]  = get_external_forces_and_torques(BEMT_config, rotor, q, qd, t)
    % External moments consists of aerodynamic forces and motor torque
    f_ext = getAerodynamicForcesAndMoments(BEMT_config,rotor, q, qd);
    motor_torque = motor_controller(q, qd, t);
    external_torques = [motor_torque;0;0;0];
end

function x_dot = odefcn(t,x, rotor, BEMT_config)
% 4 generalized variables. 
% rotor position
% teetering hinge
% positive lag-pitch hinge
% negative lag-pitch hinge
    disp(t)
    x_dot = zeros(8,1);
    q = x(1:4);
    qd = x(5:8);

    [external_torques, f_ext] = get_external_forces_and_torques(BEMT_config,rotor, q, qd, t);
    qdd = forwardDynamics(rotor, q, qd, external_torques, f_ext);
    x_dot(1:4) = qd;
    x_dot(5:8) = qdd;
end