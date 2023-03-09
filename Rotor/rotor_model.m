rotor = importrobot('RotorAssemblyFullURDF_description/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
rotor.Gravity= [0 0 -9.81];
showdetails(rotor)

numJoints = numel(homeConfiguration(rotor));

% Set up simulation parameters
tspan = [0 5];
q0 = zeros(numJoints,1);
qd0 = zeros(numJoints,1);
x0 = [q0; qd0];

[t,y] = ode45(@(t,y) odefcn(t,y,rotor), tspan, x0);
for i = 1:5:length(y)
    ax = show(rotor, y(i,1:4)', "Frames", "off", 'Preserveplot', false, 'FastUpdate', true);
    pause(0.05)
end

function external_moments = get_external_moments(q, qd)
    % External moments consists of aerodynamic forces and motor torque
    % aerodynamic_forces = bemt(q,qd);
    external_moments = [0.0005;0;0;0];
end

function x_dot = odefcn(t,x, rotor)
% 4 generalized variables. 
% rotor position
% teetering hinge
% positive lag-pitch hinge
% negative lag-pitch hinge
  x_dot = zeros(8,1);
  q = x(1:4);
  qd = x(5:8);
  external_moments = get_external_moments(q, qd);
  qdd = forwardDynamics(rotor, q, qd, external_moments);
  x_dot(1:4) = qd;
  x_dot(5:8) = qdd;
end