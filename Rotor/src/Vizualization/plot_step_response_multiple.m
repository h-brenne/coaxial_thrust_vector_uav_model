% Script to load and plot multiple data sets
data_sets = {'../Results/step026A_40hz_leishman_max_teetering.mat', ...
    '../Results/step026A_60hz_leishman_max_teetering.mat', ...
    '../Results/step026A_80hz_leishman_max_teetering.mat', ...
    '../Results/step026A_100hz_leishman_max_teetering.mat', ...
    '../Results/step026A_120hz_leishman_max_teetering.mat'}; 
% Specify different colors for each dataset, here for simplicity, we are using MATLAB's default color order, extend this if you have more datasets
colors = get(groot, 'DefaultAxesColorOrder');

figure('Units','inches', 'Position', [0 0 6.4 4.8]);  % Set the figure size (6.4x4.8 inches is MATLAB's default)
hold on;

% Initialize a cell array to store the legend handles
legend_handles = cell(size(data_sets));

for i = 1:numel(data_sets)
    load(data_sets{i});  % This loads max_values and max_times variables into workspace
    scatter(1000*(max_times-0.4), rad2deg(max_values), 30, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', colors(i,:)); % Convert to degrees if necessary
    legend_handles{i} = plot(1000*(max_times-0.4), rad2deg(max_values), 'LineWidth', 2, 'Color', colors(i,:)); % Convert to degrees if necessary and store the handle to the line object for the legend
end

xlabel('Time [ms]', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Maximum Teetering Angle [deg]', 'Interpreter', 'latex', 'FontSize', 15);
%title('Maximum Teetering Angle Over Time for Different Datasets', 'Interpreter', 'latex', 'FontSize', 16);
xlim([-50 300])
% Create the legend using the line plot handles and the dataset names
legend([legend_handles{:}], {'2400 rpm', '3600 rpm','4800 rpm','6000 rpm', '7200 rpm'}, 'Interpreter', 'latex', 'Location', 'best', 'FontSize', 12);
grid("on")
% Increase the axis label font sizes
set(gca, 'FontSize', 12);