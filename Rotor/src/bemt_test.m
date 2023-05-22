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
rpm = 5000;
pitch = 10;
Op = Oper(Uop.altitude, Uop.speed, rpm, pitch, Uflow.fluid);
OpRot = OperRotor(Rot(1), Op);
OpRot.nonDim = Sim.Misc.nonDim;

bemt(OpRot, Mod);

dT = OpRot(1,1).ElPerf.dT;
dQ = OpRot(1,1).ElPerf.dT;
plot(dT);

