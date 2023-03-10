clear

import af_tools.*
configFile = fullfile("../Rotor_Configs/TestRotor", "test_rotor.m");
[Sim, Mod, Uflow, Uop, Uaf, Ublade] = validateconfig(configFile);
Af = createairfoils(Uaf);

for i = numel(Ublade):-1:1
    Rot(i) = Rotor(Ublade(i).nBlades, Af, Ublade(i).radius, Ublade(i).chord, ...
                   Ublade(i).twist, Ublade(i).iAirfoil, Ublade(i).nElem, Ublade(i).hubPos);
    Rot(i).name = Sim.Save.filename;
    Rot(i).pitchRef = Ublade(i).pitchRef;
    Rot(i).appli = Sim.Misc.appli;
end

Mod.solver = Mod.solvers{1};
Op = Oper(Uop.altitude, Uop.speed, Uop.rpm, Uop.collective, Uflow.fluid);
OpRot = OperRotor(Rot(1), Op);
OpRot.nonDim = Sim.Misc.nonDim;

bemt(OpRot, Mod);

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

% Go from canonical frame to hinge configuration frame
% psi and beta is identical(Perhaps a sign change)

