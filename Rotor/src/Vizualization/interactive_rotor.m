clear;

rotor = importrobot('../Rotor_Configs/40DegRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';

showdetails(rotor)
gui = interactiveRigidBodyTree(rotor,MarkerScaleFactor=0.1);