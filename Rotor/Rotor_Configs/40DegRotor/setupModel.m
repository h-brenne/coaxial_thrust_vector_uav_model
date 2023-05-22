% Setup aerodynamics model
BEMT_config = initBEMTconfig('test_rotor2.m');
% Setup rotor dynamic model
rotor = importrobot('URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
%rotor.Gravity= [0 0 -9.81];
transform_positive_lag_pitch = getTransform(rotor, homeConfiguration(rotor), 'hub_1', 'side_hub_positive_1');
BEMT_config.lag_pitch_hinge_offset = transform_positive_lag_pitch(1,4);

