clear
% Setup aerodynamics model
BEMT_config = initBEMTconfig('../Rotor_Configs/TestRotor/test_rotor.m');
% Setup rotor dynamic model
rotor = importrobot('../Rotor_Configs/40DegRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
rotor.Gravity= [0 0 -9.81];
showdetails(rotor)

numJoints = numel(homeConfiguration(rotor));

% Set up simulation parameters
tspan = [0 5];
q0 = zeros(numJoints,1);
qd0 = zeros(numJoints,1);
qd0(1) = convangvel(1000, 'rpm', 'rad/s');
x0 = [q0; qd0];

[t,y] = ode45(@(t,y) odefcn(t,y,rotor, BEMT_config), tspan, x0);

save('../Results/test.mat', 't','y', 'rotor', 'BEMT_config');

function motor_torque = motor_controller(q, qd)
    % P controller for now
    Kp = 0.0005;
    
    velocity_setpoint = convangvel(1000, 'rpm', 'rad/s');;
    velocity_error = velocity_setpoint - qd(1);
    motor_torque = Kp*velocity_error;
end


function external_moments = get_external_moments(BEMT_config, q, qd)
    % External moments consists of aerodynamic forces and motor torque
    %aerodynamic_forces = getAerodynamicMoments(BEMT_config,q,qd);
    aerodynamic_forces = [0 0 0 0 0 0];
    motor_torque = motor_controller(q, qd);
    external_moments = [aerodynamic_forces(1) + motor_torque;aerodynamic_forces(2);aerodynamic_forces(3);aerodynamic_forces(4)];
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
    external_moments = get_external_moments(BEMT_config, q, qd);
    qdd = forwardDynamics(rotor, q, qd, external_moments);
    x_dot(1:4) = qd;
    x_dot(5:8) = qdd;
end