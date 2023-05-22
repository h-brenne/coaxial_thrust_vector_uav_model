function M = getAerodynamicMoments_old(BEMT_config,q,q_d)
%getAerodynamicMoments Get aerodynamic moments from BEMT
% TODO: cleanup input data. Calculate for both blades. Correct frames.

% Positive side hub
OpRot = BEMT_config.OpRot;
rpm = convangvel(q_d(1), 'rad/s', 'rpm');
collective_pitch = max(0.01, 10); %- 0.5*rad2deg(q(3))); % TODO: Fix

Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, collective_pitch, OpRot.Op.Flow.fluid);
OpRot = OperRotor(OpRot.Rot, Op);
bemt(OpRot, BEMT_config.Mod);

dT = OpRot(1,1).ElPerf.dT;
dQ = OpRot(1,1).ElPerf.dT;

% Calculate moments in canonical frame
M_psi_1 = 0;
M_zeta_1 = 0;
M_beta_1 = 0;
for i = 1:length(dT)
    section_radius = OpRot.Rot.cutout + i*OpRot.Rot.Bl.dy;
    hinge_distance = OpRot.Rot.cutout; % TODO: fix
    M_psi_1 = M_psi_1 - section_radius*dQ(i);
    M_zeta_1= M_zeta_1 - (section_radius-hinge_distance)*dQ(i);
    M_beta_1 = M_beta_1 + section_radius*dT(i);
end

% Negative side hub
collective_pitch = max(0.01,10);% + 0.5*rad2deg(q(4))); % TODO: Fix

Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, collective_pitch, OpRot.Op.Flow.fluid);
OpRot = OperRotor(OpRot.Rot, Op);
bemt(OpRot, BEMT_config.Mod);

dT = OpRot(1,1).ElPerf.dT;
dQ = OpRot(1,1).ElPerf.dT;

% Calculate moments in canonical frame
M_psi_2 = 0;
M_zeta_2 = 0;
M_beta_2 = 0;
for i = 1:length(dT)
    section_radius = OpRot.Rot.cutout + i*OpRot.Rot.Bl.dy;
    hinge_distance = OpRot.Rot.cutout; % TODO: fix
    M_psi_2 = M_psi_2 - section_radius*dQ(i);
    M_zeta_2 = M_zeta_2 - (section_radius-hinge_distance)*dQ(i);
    M_beta_2 = M_beta_2 + section_radius*dT(i);
end

M = [M_psi_1 + M_psi_2; -M_beta_1 + M_beta_2; M_zeta_1; M_zeta_2];
end

