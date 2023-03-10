clear
% Setup aerodynamics model
BEMT_config = initBEMTconfig("test_rotor.m");
% Setup rotor dynamic model
rotor = importrobot('../Rotor_Configs/TestRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
rotor.Gravity= [0 0 -9.81];
showdetails(rotor)

numJoints = numel(homeConfiguration(rotor));

% Set up simulation parameters
tspan = [0 0.02];
q0 = zeros(numJoints,1);
qd0 = zeros(numJoints,1);
qd0(1) = convangvel(3000, 'rpm', 'rad/s');
x0 = [q0; qd0];

% ode45 and ode15s is slow
[t,y] = ode45(@(t,y) odefcn(t,y,rotor, BEMT_config), tspan, x0);
for i = 1:2:length(y)
    ax = show(rotor, y(i,1:4)', "Frames", "off", 'Preserveplot', false, 'FastUpdate', true);
    pause(0.05)
end

function external_moments = get_external_moments(BEMT_config, q, qd)
    % External moments consists of aerodynamic forces and motor torque
    aerodynamic_forces = getAerodynamicMoments(BEMT_config,q,qd);
    external_moments = [aerodynamic_forces(1);aerodynamic_forces(2);aerodynamic_forces(3);aerodynamic_forces(3)];
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