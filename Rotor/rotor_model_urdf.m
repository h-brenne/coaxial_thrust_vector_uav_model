rotor = importrobot('RotorAssemblyFullURDF_description/urdf/Rotor.urdf');
rotor.DataFormat = 'row';
massMatrix(rotor)
showdetails(rotor)

config = randomConfiguration(rotor);
show(rotor, config, "Frames", "off")
%gui = interactiveRigidBodyTree(rotor,"MarkerScaleFactor",0.25);
