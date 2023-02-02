clc; close all; clear all;

dt = 0.002;
t_end = 20;
tspan = [0 t_end-dt];
t = 0:dt:t_end-dt;


model = CoaxialRotorcraftModel(@(t, x) external_input_controller(t, x));

initial_height = 3;
y_0 = zeros(12, 1);
y_0(1) = 3; % n
y_0(2) = 0; % e
y_0(3) = -initial_height; % d
y_0(10) = deg2rad(30); % theta
y_0(11) = deg2rad(30); % phi
y_0(12) = deg2rad(30); % psi

% Simulate model
[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);
north = y_sim(:,1);
east = y_sim(:,2);
heigth = -y_sim(:,3);

phi = y_sim(:,10);
theta = y_sim(:,11);
psi = y_sim(:,12);
F_x = zeros(length(t_sim),1);
F_y = zeros(length(t_sim),1);
F_z = zeros(length(t_sim),1);
M_x = zeros(length(t_sim),1);
M_y = zeros(length(t_sim),1);
M_z = zeros(length(t_sim),1);

u_trajectory = zeros(length(t_sim), 6);
for i = 1:length(t_sim)
    u_trajectory(i,:) = external_input_controller(t_sim(i), y_sim(i, :));
    [F_x(i), F_y(i), F_z(i), M_x(i), M_y(i), M_z(i)] = model.bound_input(u_trajectory(i, 1), u_trajectory(i, 2), u_trajectory(i, 3), u_trajectory(i, 4), u_trajectory(i, 5), u_trajectory(i,6));
end

figure(1)
subplot(3,1,1)
plot(t_sim, north);
legend('north')
subplot(3,1,2)
plot(t_sim, east);
legend('east')
subplot(3,1,3)
plot(t_sim, heigth);
legend('heigth')

figure(2)
subplot(3,1,1)
plot(t_sim, F_x);
legend('F_x')
subplot(3,1,2)
plot(t_sim, F_y);
legend('F_y')
subplot(3,1,3)
plot(t_sim, F_z);
legend('F_z')

figure(3)

subplot(3,1,1)
plot(t_sim, M_x);
legend('M_x')
subplot(3,1,2)
plot(t_sim, M_y);
legend('M_y')
subplot(3,1,3)
plot(t_sim, M_z);
legend('M_z')

figure(4)
subplot(3,1,1)
plot(t_sim, rad2deg(phi));
legend('roll')
subplot(3,1,2)
plot(t_sim, rad2deg(theta));
legend('pitch')
subplot(3,1,3)
plot(t_sim, rad2deg(psi));
legend('yaw')
% Visualize trajectory
visualizer = AircraftVisualizer;
visualizer.plot_trajectory(t_sim, y_sim, u_trajectory);

function u = constant_input(t, x)
    u(1) = 0;
    u(2) = 0;
    u(3) = 0;
    u(4) = 0;
    u(5) = 0;
    u(6) = 1;
end

function u = external_input_controller(t, x)
    F_des = position_controller(t, x);
    M_des = attitude_controller(t, x);
    
    u(1) = F_des(1);
    u(2) = F_des(2);
    u(3) = F_des(3);
    u(4) = M_des(1);
    u(5) = M_des(2);
    u(6) = M_des(3);
end

function F_des = position_controller(t, x)
    % Unpack states
    x = num2cell(x);
    [n, e, d, u, v, w, p, q, r, phi, theta, psi] = x{:};
    
    K_p = 20;
    K_d = 5;
    
    world_pos_des = [0 0 0]';
    
    R = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(psi)*sin(theta);
                 cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                 -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
    world_pos_dot = R*[u v w]';
    % Force in world frame, P-controller
    Fw_x = K_p*(world_pos_des(1)-n)-K_d*world_pos_dot(1);
    Fw_y = K_p*(world_pos_des(2)-e)-K_d*world_pos_dot(2);
    Fw_z = K_p*(world_pos_des(3)-d)-K_d*world_pos_dot(3); 
    
    
    
    % Force in body frame       
    F_des = R'*[Fw_x Fw_y Fw_z]';
end
function M_des = attitude_controller(t, x)
    % Unpack states
    x = num2cell(x);
    [n, e, d, u, v, w, p, q, r, phi, theta, psi] = x{:};
    
    K_p = 2;
    K_d = 3;
    R_des = [0 0 0];
    
    M_x = K_p*(R_des(1)-phi - K_d*p);
    M_y = K_p*(R_des(2)-theta - K_d*q);
    M_z = K_p*(R_des(3)-psi) - K_d*r;
    M_des = [M_x M_y M_z]';
end