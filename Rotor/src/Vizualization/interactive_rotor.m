clear;

rotor = importrobot('../Rotor_Configs/40DegRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
q = [0; 0; deg2rad(20); deg2rad(20)];
transform_positive_lag_pitch = getTransform(rotor, homeConfiguration(rotor), 'hub_1', 'side_hub_positive_1');
lag_pitch_hinge_offset = transform_positive_lag_pitch(1,4);


showdetails(rotor)
disp(rotor)
show(rotor, q)
%gui = interactiveRigidBodyTree(rotor,MarkerScaleFactor=0.001);