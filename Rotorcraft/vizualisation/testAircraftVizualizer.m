clc; close all; clear all;
dt = 1;
t_end = 2;
tspan = [0 t_end-dt];
t = 0:dt:t_end-dt;

y = zeros(length(t), 12);
y(:,10) = deg2rad(0); % phi
y(:,11) = deg2rad(0); % theta
y(:,12) = deg2rad(45); % psi
u_trajectory = zeros(length(t), 6);
u_trajectory(:, 3) = -50;
visualizer = AircraftVisualizer;
visualizer.plot_trajectory(t, y, u_trajectory);