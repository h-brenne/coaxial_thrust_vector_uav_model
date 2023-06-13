clear;
experiment = 'step026A_80hz_leishman';
save_data = false;
load(strcat('../Results/', experiment, '.mat'));

% Get the default text interpreter
default_interpreter = get(0, 'DefaultTextInterpreter');

% Change the default text interpreter to latex
set(0, 'DefaultTextInterpreter', 'latex');

total_thrust = [OpRot_positive.thrust] + [OpRot_negative.thrust];
total_torque = [OpRot_positive.torque] + [OpRot_negative.torque];

motor_angle = mod(y(:, 1), 2*pi);
motor_speed = y(:,5);
teetering_angle = y(:,2);
side_hub_positive_angle = y(:,3);
side_hub_negative_angle = y(:,4);

% Given parameters
step_length_s = 0.4;
min_amplitude = 0.0;
max_amplitude = 0.26;
amplitude_step = 0.26;
transient_wait_s = 0.1;

% Compute input_amplitude
input_amplitude = min_amplitude:amplitude_step:max_amplitude;

% Preallocate time_sets
num_sets = numel(input_amplitude);
time_sets = zeros(num_sets, 2);

% Preallocate arrays for phases and amplitudes
amp_speeds = zeros(size(time_sets, 1), 1);
phase_speeds = zeros(size(time_sets, 1), 1);
amp_teeters = zeros(size(time_sets, 1), 1);
phase_teeters = zeros(size(time_sets, 1), 1);
amp_poss = zeros(size(time_sets, 1), 1);
phase_poss = zeros(size(time_sets, 1), 1);
amp_negs = zeros(size(time_sets, 1), 1);
phase_negs = zeros(size(time_sets, 1), 1);

% Compute time_sets
for i = 1:num_sets
    time_sets(i, :) = [(i-1)*step_length_s + transient_wait_s, i*step_length_s - 0.5*transient_wait_s];
end

colorOrder = get(gca, 'ColorOrder');

% Start the figures for the individual variables
figure(1); title('Motor Speed', 'Interpreter', 'latex');
xlabel('Motor Angle (deg)', 'Interpreter', 'latex');
ylabel('Motor Speed (rad/s)', 'Interpreter', 'latex');
hold on;

figure(2); title('Teetering Angle', 'Interpreter', 'latex');
xlabel('Motor Angle (deg)', 'Interpreter', 'latex');
ylabel('Teetering Angle (deg)', 'Interpreter', 'latex');
hold on;

figure(3); title('Side Hub Positive Angle', 'Interpreter', 'latex');
xlabel('Motor Angle (deg)', 'Interpreter', 'latex');
ylabel('Side Hub Positive Angle (deg)', 'Interpreter', 'latex');
hold on;

figure(4); title('Side Hub Negative Angle', 'Interpreter', 'latex');
xlabel('Motor Angle (deg)', 'Interpreter', 'latex');
ylabel('Side Hub Negative Angle (deg)', 'Interpreter', 'latex');
hold on;

% Start the figures for the phases
figure(5); title('Motor Speed Phase', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Phase (deg)', 'Interpreter', 'latex');
hold on;

figure(6); title('Teetering Phase', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Phase (deg)', 'Interpreter', 'latex');
hold on;

figure(7); title('Side Hub Positive Phase', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Phase (deg)', 'Interpreter', 'latex');
hold on;

figure(8); title('Side Hub Negative Phase', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Phase (deg)', 'Interpreter', 'latex');
hold on;

% Start the figures for the amplitudes
figure(9); title('Motor Speed Amplitude', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
hold on;

figure(10); title('Teetering Amplitude', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
hold on;

figure(11); title('Side Hub Positive Amplitude', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
hold on;

figure(12); title('Side Hub Negative Amplitude', 'Interpreter', 'latex');
xlabel('Input Amplitude', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
hold on;


% Loop over time sets
for idx = 1:size(time_sets, 1)
    time_min = time_sets(idx, 1);
    time_max = time_sets(idx, 2);
    
    [selected_motor_angle, selected_motor_speed, selected_teetering_angle, selected_side_hub_positive_angle, selected_side_hub_negative_angle, amp_speed, phase_speed, amp_teeter, phase_teeter, amp_pos, phase_pos, amp_neg, phase_neg] = analyzeData(y, t, time_min, time_max);
    
    % Create scatter plots with motor angle on the x-axis and each variable on the y-axis
    figure(1);  % switch to the figure for Motor Speed
    scatter(rad2deg(selected_motor_angle), selected_motor_speed, 'DisplayName', ['Input Amplitude ', num2str(input_amplitude(idx))], 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    legend('show', 'Interpreter', 'latex');
    xticks(0:30:360);

    figure(2);  % switch to the figure for Teetering Angle
    scatter(rad2deg(selected_motor_angle), rad2deg(selected_teetering_angle), 'DisplayName', ['Input Amplitude ', num2str(input_amplitude(idx))], 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    legend('show', 'Interpreter', 'latex');
    xticks(0:30:360);
    
    % Plot phases against input amplitude
    figure(5);  % switch to the figure for Phases
    scatter(input_amplitude(idx), rad2deg(phase_speed), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(6);
    scatter(input_amplitude(idx), rad2deg(phase_teeter), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(7);
    scatter(input_amplitude(idx), rad2deg(phase_pos), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(8);
    scatter(input_amplitude(idx), rad2deg(phase_neg), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    
    % Plot amplitudes against input amplitude
    figure(9);  % switch to the figure for Amplitudes
    scatter(input_amplitude(idx), rad2deg(amp_speed), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(10);
    scatter(input_amplitude(idx), rad2deg(amp_teeter), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(11);
    scatter(input_amplitude(idx), rad2deg(amp_pos), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    figure(12);
    scatter(input_amplitude(idx), rad2deg(amp_neg), 'MarkerFaceColor', colorOrder(mod(idx-1, size(colorOrder, 1))+1, :));
    if save_data
        % Save amplitudes and phases to arrays
        amp_speeds(idx) = amp_speed;
        phase_speeds(idx) = rad2deg(phase_speed);
        amp_teeters(idx) = rad2deg(amp_teeter);
        phase_teeters(idx) = rad2deg(phase_teeter);
        amp_poss(idx) = rad2deg(amp_pos);
        phase_poss(idx) = rad2deg(phase_pos);
        amp_negs(idx) = rad2deg(amp_neg);
        phase_negs(idx) = rad2deg(phase_neg);
    
        % Create a table from selected data
        T_selected = table(rad2deg(selected_motor_angle), selected_motor_speed,...
            rad2deg(selected_teetering_angle), rad2deg(selected_side_hub_positive_angle),...
            rad2deg(selected_side_hub_negative_angle), ...
            'VariableNames', {'Motor_Angle', 'Motor_Speed', 'Teetering_Angle', 'Side_Hub_Positive_Angle', 'Side_Hub_Negative_Angle'});
        
        % Create a filename with the corresponding input amplitude value
        filename = ['../Results/' experiment '_amp_' num2str(input_amplitude(idx)) '.csv'];
        
        % Write the table to a CSV file
        writetable(T_selected, filename);
    end
end
if save_data
    % Convert arrays to table
    T = table(input_amplitude', amp_speeds, phase_speeds, amp_teeters, phase_teeters, amp_poss, phase_poss, amp_negs, phase_negs, ...
        'VariableNames', {'Input_Amplitude', 'Amp_Speed', 'Phase_Speed', 'Amp_Teeter', 'Phase_Teeter', 'Amp_Pos', 'Phase_Pos', 'Amp_Neg', 'Phase_Neg'});
    
    % Save table to CSV

    writetable(T, strcat('../Results/', experiment, '.csv'));
end
figure;
plot(t, motor_speed)
legend('Motor speed')
hold on
plot(t, omega + omega_tilde)
legend('Motor speed setpoint')
figure;
plot(t, total_thrust)
legend('Total thrust')
figure;
plot(t, total_torque)
legend('Total Torque')
figure;
plot(t, rad2deg(y(:,2)))
legend('Teetering angle [deg]')
figure;
plot(t, rad2deg(y(:,3)))
legend('Side Hub Positive angle [deg]')
figure;
plot(t, rad2deg(y(:,4)))
legend('Side Hub Negative angle [deg]')
figure;
scatter(selected_motor_angle, selected_motor_speed)
xlabel('Motor Angle (rad)');
ylabel('Motor Speed (rad/s)');
figure;
scatter(selected_motor_angle, rad2deg(selected_teetering_angle))
ylabel('Teetering Angle (deg)');
xlabel('Motor Angle (rad)');
figure;
scatter(selected_motor_angle, rad2deg(selected_side_hub_positive_angle))
ylabel('SideHubPosittive Angle (deg)');
xlabel('Motor Angle (rad)');
figure;
plot(t, motor_torque)
ylabel('Motor Torque [Nm]')

function [selected_motor_angle, selected_motor_speed, selected_teetering_angle,...
        selected_side_hub_positive_angle, selected_side_hub_negative_angle,...
        amp_speed, phase_speed, amp_teeter, phase_teeter, amp_pos, phase_pos,...
        amp_neg, phase_neg] = analyzeData(y, t, time_min, time_max)
    % Select data based on the time mask
    mask = (t >= time_min) & (t <= time_max);

    % Extract the variables
    motor_angle = mod(y(:, 1), 2*pi);
    motor_speed = y(:,5);
    teetering_angle = y(:,2);
    side_hub_positive_angle = y(:,3);
    side_hub_negative_angle = y(:,4);

    % Select the data
    selected_motor_angle = motor_angle(mask);
    selected_motor_speed = motor_speed(mask);
    selected_teetering_angle = teetering_angle(mask);
    selected_side_hub_positive_angle = side_hub_positive_angle(mask);
    selected_side_hub_negative_angle = side_hub_negative_angle(mask);

    % Define bins for motor angle
    num_bins = 360;
    bin_edges = linspace(min(selected_motor_angle), max(selected_motor_angle), num_bins+1);
    
    % Average the selected data over the motor angle bins
    avg_motor_speed = accumarray(discretize(selected_motor_angle, bin_edges), selected_motor_speed, [], @mean);
    avg_teetering_angle = accumarray(discretize(selected_motor_angle, bin_edges), selected_teetering_angle, [], @mean);
    avg_pos = accumarray(discretize(selected_motor_angle, bin_edges), selected_side_hub_positive_angle, [], @mean);
    avg_neg = accumarray(discretize(selected_motor_angle, bin_edges), selected_side_hub_negative_angle, [], @mean);
    
    % Calculate the amplitude and phase
    amp_speed = (max(avg_motor_speed) - min(avg_motor_speed)) / 2;
    phase_speed = bin_edges(avg_motor_speed == max(avg_motor_speed));
    
    amp_teeter = (max(avg_teetering_angle) - min(avg_teetering_angle)) / 2;
    phase_teeter = bin_edges(avg_teetering_angle == max(avg_teetering_angle));
    
    amp_pos = (max(avg_pos) - min(avg_pos)) / 2;
    phase_pos = bin_edges(avg_pos == max(avg_pos));
    
    amp_neg = (max(avg_neg) - min(avg_neg)) / 2;
    phase_neg = bin_edges(avg_neg == max(avg_neg));
end