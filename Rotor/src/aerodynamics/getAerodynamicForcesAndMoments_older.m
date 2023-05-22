function [f_ext, OpRot_positive, OpRot_negative] = getAerodynamicForcesAndMoments_old(BEMT_config, rotor, q,q_d)
%getAerodynamicMoments Get aerodynamic forces and moments in base frame

% Positive side hub
OpRot = BEMT_config.OpRot;
rpm = 0;%convangvel(q_d(1), 'rad/s', 'rpm');
%collective_pitch = max(0.01, 10); % + 0.5*rad2deg(q(3))); % TODO: Fix

% Compute Jacobian
J = geometricJacobian(rotor, q, 'side_hub_positive_1');
% Compute velocity in global frame
v_global = J * q_d;
% Get the end effector pose
T = getTransform(rotor, q, 'side_hub_positive_1');
% Extract the rotation matrix
R = T(1:3,1:3);
% Compute velocity in body frame
v_body_angular = R.' * v_global(1:3);
v_body_linear = R.' * v_global(4:6);
% Set section tangential velocities

% Distance of elements along x-axis
% The total length is the radius of the propeller minus
% root cutout. It is assumed now that the root cutout
% and side_hub frame origin coincides
r = linspace(0, OpRot.Rot.Bl.y(end)-OpRot.Rot.Bl.y(1), OpRot.Rot.Bl.nElem).';
r_mat = [r, zeros(length(r), 2)]; % Creating a matrix with each row as [r_i 0 0]
cross_product_result = cross(repmat(v_body_angular.', length(r), 1), r_mat);
% For the positive hub, y-axis points opposite to nominal direction 
y_velocities = -(v_body_linear(2) + cross_product_result(:,2));
z_velocities = (v_body_linear(3) + cross_product_result(:,3));

% Problem: axial_inflow must be postive... Why?

% TODO: Send blade section velocities to BEMT
Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, OpRot.Op.coll, OpRot.Op.Flow.fluid);
OpRot_positive = OperRotor(OpRot.Rot, Op);
OpRot_positive.ElPerf.tgSpeed = abs(y_velocities);
BEMT_config.Mod.axial_inflows = z_velocities;
if (all(y_velocities > 0.02)  &&  all(abs(z_velocities) > 0.2))
    bemt(OpRot_positive, BEMT_config.Mod);
end

dT = OpRot_positive(1,1).ElPerf.dT;
dQ = OpRot_positive(1,1).ElPerf.dQ;
% dQ is multiplied by the radial position assumed by Rotare, which are
% slightly different than ours due to hinge angles. Needs to be negated due
% to flipped side_hub_positive_1 y-axis
% Compute distributed forces and moments
dFz = dT;
dFy = -dQ/OpRot.Rot.Bl.dy;

% Compute net force and moments
Fy = trapz(r, dFy); % force along y-axis
Fz = trapz(r, dFz); % force along z-axis
Torque_y = trapz(r, dFz.*r'); % moment about y-axis
Torque_z = trapz(r, dFy.*r'); % moment about z-axis

% Compose the wrench
wrench_positive = [0; Torque_y; Torque_z; 0; Fy; Fz]

%% Negative side hub
%collective_pitch = max(0.01, 10);% - 0.5*rad2deg(q(4))); % TODO: Fix
% Compute Jacobian
J = geometricJacobian(rotor, q, 'side_hub_negative_1');
% Compute velocity in global frame
v_global = J * q_d;
% Get the end effector pose
T = getTransform(rotor, q, 'side_hub_negative_1');
% Extract the rotation matrix
R = T(1:3,1:3);
% Compute velocity in body frame
v_body_angular = R.' * v_global(1:3);
v_body_linear = R.' * v_global(4:6);
% Set section tangential velocities

% Distance of elements along x-axis
% The total length is the radius of the propeller minus
% root cutout. It is assumed now that the root cutout
% and side_hub frame origin coincides
r = linspace(0, OpRot.Rot.Bl.y(end)-OpRot.Rot.Bl.y(1), OpRot.Rot.Bl.nElem).';
r_mat = [r, zeros(length(r), 2)]; % Creating a matrix with each row as [r_i 0 0]
cross_product_result = cross(repmat(v_body_angular.', length(r), 1), r_mat);
% For the positive hub, y-axis points opposite to nominal direction 
y_velocities = abs(v_body_linear(2) + cross_product_result(:,2));
z_velocities = (v_body_linear(3) + cross_product_result(:,3));
% Problem: axial_inflow must be postive... Why?
%BEMT_config.Mod.axial_inflows = z_velocities;

% TODO: Send blade section velocities to BEMT
Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, OpRot.Op.coll, OpRot.Op.Flow.fluid);
OpRot_negative = OperRotor(OpRot.Rot, Op);
OpRot_negative.ElPerf.tgSpeed = y_velocities;
BEMT_config.Mod.axial_inflows = z_velocities;
if (all(y_velocities > 0.02)  &&  all(abs(z_velocities) > 0.02))
    bemt(OpRot_negative, BEMT_config.Mod);
end

dT = OpRot_positive(1,1).ElPerf.dT;
dQ = OpRot_positive(1,1).ElPerf.dQ;
% dQ is multiplied by the radial position assumed by Rotare, which are
% slightly different than ours due to hinge angles.
% Compute distributed forces and moments
dFz = dT;
dFy = dQ/OpRot.Rot.Bl.dy;

% Compute net force and moments
Fy = trapz(r, dFy); % force along y-axis
Fz = trapz(r, dFz); % force along z-axis
Torque_y = trapz(r, dFz.*r'); % moment about y-axis
Torque_z = trapz(r, dFy.*r'); % moment about z-axis

% Compose the wrench
wrench_negative = [0; Torque_y; Torque_z; 0; Fy; Fz]

f_ext_1 = externalForce(rotor,'side_hub_positive_1',wrench_positive, q);
% side_hub_negative_1 frame is flipped as of now
f_ext_2 = externalForce(rotor,'side_hub_negative_1',wrench_negative, q);
% Can f_ext be summed like this?
f_ext = f_ext_1 + f_ext_2;
end

