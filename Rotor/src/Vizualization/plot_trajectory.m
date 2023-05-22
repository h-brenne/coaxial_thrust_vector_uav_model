clear;
BEMT_config = initBEMTconfig('../Rotor_Configs/40DegRotor/BEMTconfig.m');
load('../Results/test.mat');
rotor = importrobot('../Rotor_Configs/40DegRotor/URDF/urdf/Rotor.urdf');
rotor.DataFormat = 'column';
rotor.Gravity= [0 0 -9.81];
% Set parameter needed for aerodynamics
transform_positive_lag_pitch = getTransform(rotor, homeConfiguration(rotor), 'hub_1', 'side_hub_positive_1');
BEMT_config.lag_pitch_hinge_offset = transform_positive_lag_pitch(1,4);

skip_steps = 20;
for i= 1:skip_steps:length(y(:,1))
    disp(i);
    [f_ext, OpRot_positive(i:i+skip_steps), OpRot_negative(i:i+skip_steps)] = getAerodynamicForcesAndMoments(BEMT_config, rotor, y(i,1:4)', y(i,5:8)');
end

OpRot_positive = OpRot_positive(1:length(y(:,1)));
OpRot_negative = OpRot_negative(1:length(y(:,1)));
total_thrust = [OpRot_positive.thrust] + [OpRot_negative.thrust];
total_torque = [OpRot_positive.torque] + [OpRot_negative.torque];

motor_angle = mod(y(:, 1), 2*pi);
motor_speed = y(:,5);
teetering_angle = y(:,2);
side_hub_positive_angle = y(:,3);
side_hub_negative_angle = y(:,4);
mask = (t >= 0.5) & (t <= 0.9);
selected_motor_angle = motor_angle(mask);
selected_motor_speed = motor_speed(mask);
selected_teetering_angle = teetering_angle(mask);
selected_side_hub_positive_angle = side_hub_positive_angle(mask);
selected_side_hub_negative_angle = side_hub_negative_angle(mask);

figure(1);
plot(t, y(:,5))
legend('Hub speed')
figure(2);
plot(t, total_thrust)
legend('Total thrust')
figure(3);
plot(t, total_torque)
legend('Total Torque')
figure(4)
plot(t, rad2deg(y(:,2)))
legend('Teetering angle [deg]')
figure(5)
plot(t, rad2deg(y(:,3)))
legend('Side Hub Positive angle [deg]')
figure(6)
plot(t, rad2deg(y(:,4)))
legend('Side Hub Negative angle [deg]')
figure(7)
scatter(selected_motor_angle, selected_motor_speed)
xlabel('Motor Angle (rad)');
ylabel('Motor Speed (rad/s)');
figure(8)
scatter(selected_motor_angle, rad2deg(selected_teetering_angle))
ylabel('Teetering Angle (deg)');
xlabel('Motor Angle (rad)');
figure(9)
scatter(selected_motor_angle, rad2deg(selected_side_hub_positive_angle))
ylabel('SideHubPosittive Angle (deg)');
xlabel('Motor Angle (rad)');