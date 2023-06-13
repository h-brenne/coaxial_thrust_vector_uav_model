clear;
experiment = 'step026A_80hz_leishman';
save_data = false;
load(strcat('../Results/', experiment, '.mat'));

% Get the default text interpreter
default_interpreter = get(0, 'DefaultTextInterpreter');
colors = get(groot, 'DefaultAxesColorOrder');

% Change the default text interpreter to latex
set(0, 'DefaultTextInterpreter', 'latex');

total_thrust = [OpRot_positive.thrust] + [OpRot_negative.thrust];
total_torque = [OpRot_positive.torque] + [OpRot_negative.torque];

motor_angle = mod(y(:, 1), 2*pi);
teetering_angle = y(:,2);

% Initialize
max_values = []; 
max_times = []; 
current_max = -inf; 
current_max_time = t(1);

for i = 2:length(motor_angle)
    % If a new rotation starts
    if motor_angle(i) < motor_angle(i-1)
        % Store maximum of last rotation
        max_values = [max_values; current_max]; 
        max_times = [max_times; current_max_time];
        
        % Reset maximum
        current_max = -inf; 
    end
    
    % Update maximum
    if teetering_angle(i) > current_max
        current_max = teetering_angle(i);
        current_max_time = t(i);
    end
end

% Catch the last rotation if it wasn't caught inside the loop
if current_max > -inf
    max_values = [max_values; current_max]; 
    max_times = [max_times; current_max_time];
end

save(strcat('../Results/', experiment, '_max_teetering.mat'), 'max_values', 'max_times');

% Plot
figure;
plot(1000*(t-0.4), rad2deg(teetering_angle)); % Convert to degrees if necessary
hold on;
scatter(1000*(max_times-0.4), rad2deg(max_values), 'r', 'filled', 'MarkerFaceColor', colors(3,:), 'MarkerEdgeColor', colors(3,:)); % Convert to degrees if necessary
plot(1000*(max_times-0.4), rad2deg(max_values), 'r--', 'LineWidth', 1, 'Color', colors(3,:)); % Convert to degrees if necessary
xlabel('Time [ms]', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Teetering Angle [deg]', 'Interpreter', 'latex', 'FontSize', 15);
legend('Teetering Angle', 'Maximum Per Revolution', 'Interpreter', 'latex', 'Location', 'east', 'FontSize', 12);
set(gca, 'FontSize', 12);
xlim([-50 300])
grid("on")