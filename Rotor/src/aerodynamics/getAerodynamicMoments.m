function M = getAerodynamicMoments(BEMT_config,q,q_d)
%getAerodynamicMoments Get aerodynamic moments from BEMT
% TODO: cleanup input data. Calculate for both blades. Correct frames.
OpRot = BEMT_config.OpRot;
rpm = 3000; %convangvel(q_d(1), 'rad/s', 'rpm');
collective_pitch = 10;%0.5*rad2deg(q(3)); % TODO: Fix

Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, collective_pitch, OpRot.Op.Flow.fluid);
OpRot = OperRotor(OpRot.Rot, Op);
bemt(OpRot, BEMT_config.Mod);

dT = OpRot(1,1).ElPerf.dT;
dQ = OpRot(1,1).ElPerf.dT;

% Calculate moments in canonical frame
M_psi = 0;
M_zeta = 0;
M_beta = 0;
for i = 1:length(dT)
    section_radius = OpRot.Rot.cutout + i*OpRot.Rot.Bl.dy;
    hinge_distance = OpRot.Rot.cutout; % TODO: fix
    M_psi = M_psi - section_radius*dQ(i);
    M_zeta = M_zeta + (section_radius-hinge_distance)*dQ(i);
    M_beta = M_beta + section_radius*dT(i);
end
% TODO: Go from canonical frame to hinge configuration frame
% psi and beta is identical(Perhaps a sign change
M = [M_psi; M_zeta; M_beta];
end

