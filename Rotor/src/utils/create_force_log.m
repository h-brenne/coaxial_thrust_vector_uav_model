clear;
experiment = 'multi_amp_80';
load(strcat('../Results/', experiment, '.mat'));



skip_steps = 30;

force = zeros(3, length(t));
torque = zeros(3, length(t));
for i = 1:skip_steps:length(y(:,1))
    disp(t(i))

    q = y(i, 1:4)';
    qd = y(i, 5:8)';
    e = y(i, 9);
    speed_setpoint = omega(i) + omega_tilde(i);

    [base_force, base_torque] = calculateTotalFext(BEMT_config, rotor, q,...
        qd, e, omega(i), speed_setpoint);
    force(:,i:i+skip_steps) = repmat(base_force, 1, skip_steps+1);
    torque(:,i:i+skip_steps) = repmat(base_torque, 1, skip_steps+1);
end

force = force(:,1:length(t));
torque = torque(:,1:length(t));

window = 1;

force(1,:) = movmean(force(1,:), window);
force(2,:) = movmean(force(2,:), window);
force(3,:) = movmean(force(3,:), window);

torque(1,:) = movmean(torque(1,:), window);
torque(2,:) = movmean(torque(2,:), window);
torque(3,:) = movmean(torque(3,:), window);

figure(1);
plot(t, force(1, :))
hold on
plot(t, force(2, :))
hold on
plot(t, force(3, :))
legend('Force X','Force Y','Force Z')

figure(2);
plot(t, torque(1, :))
hold on
plot(t, torque(2, :))
hold on
plot(t, torque(3, :))
legend('Torque X','Torque Y','Torque Z')

function [baseForce, baseTorque] = calculateTotalFext(BEMT_config, rotor, q, qd, e, omega, speed_setpoint)
    % We want to find force and moment in body frame 

    motor_torque = pi_motor_controller(qd(1), e, speed_setpoint);
    f_ext = getAerodynamicForcesAndMoments(BEMT_config,rotor, q, qd, omega);
    external_torques = [motor_torque;0;0;0];
    qdd = forwardDynamics(rotor, q, qd, external_torques, f_ext);
    jointTorq = inverseDynamics(rotor,q,qd,qdd);
    
    bodyNames = rotor.BodyNames;
    numberOfBodies = length(bodyNames);
    
    % Initialize total base force and torque as zero
    
    baseForce = zeros(3, 1);
    baseTorque = [motor_torque; 0; 0];
    % Iterate over all bodies
    for bodyIdx = 1:numberOfBodies
        % Get the Jacobian for this body
        J = geometricJacobian(rotor, q, bodyNames{bodyIdx});
    
        % Compute the body frame torques from joint torques
        bodyWrench = J'\jointTorq;
        
        bodyTorque = bodyWrench(1:3);
        % Extract external force and torque for the current body in body frame
        % These is supposed to be given in base frame
        extForce_base = f_ext(4:6, bodyIdx);
        extTorque_base = f_ext(1:3, bodyIdx);
    
        % Get the rotation part of the transform from base frame to body frame
        %T = getTransform(rotor, q, bodyNames{bodyIdx});
        %R = T(1:3, 1:3);
    
        % Transform external force and torque to the base frame
        %extForce_base = R * extForce_body;
        %extTorque_base = R * extTorque_body;
    
        % Add to the total base force and torque
        baseForce = baseForce + extForce_base;
        baseTorque = baseTorque + extTorque_base + bodyTorque;
    end

end