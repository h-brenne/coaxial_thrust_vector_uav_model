function BEMT_config = initBEMTconfig(filename)
%initBEMTconfig Summary of this function goes here
% Setup aerodynamics model
import af_tools.*
configFile = fullfile(filename);
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
BEMT_config.Mod = Mod;
BEMT_config.OpRot = OpRot;
end

