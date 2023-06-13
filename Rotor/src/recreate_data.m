function [OpRot_positive, OpRot_negative, motor_torque, omega, amplitude, phase, omega_tilde] = recreate_data(...
    BEMT_config, rotor, y, t, omegas, amplitudes, phases, command_step_length, skip_steps)
    for i = 1:skip_steps:length(y(:,1))
        disp(t(i));
        if t(i)== 0
            current_step = 1;
        else
            current_step = ceil((t(i)) / command_step_length);
        end

        omega(i:i+skip_steps) = omegas(current_step);
        amplitude(i:i+skip_steps) = amplitudes(current_step);
        phase(i:i+skip_steps) = phases(current_step);
        
        omega_tilde(i:i+skip_steps) = sinusoidal_control_law(y(i,1), omega(i),...
            amplitude(i), phase(i));
        speed_setpoint = (omega(i) + omega_tilde(i)); 
    
        motor_torque(i:i+skip_steps) = pi_motor_controller(y(i,5), y(i,9), speed_setpoint);
        [~, OpRot_positive(i:i+skip_steps), OpRot_negative(i:i+skip_steps)] = ...
            getAerodynamicForcesAndMoments(BEMT_config, rotor, y(i,1:4)',...
            y(i,5:8)', omegas(current_step));
    end

    OpRot_positive = OpRot_positive(1:length(y(:,1)));
    OpRot_negative = OpRot_negative(1:length(y(:,1)));
    motor_torque = motor_torque(1:length(y(:,1)));
    omega_tilde = omega_tilde(1:length(y(:,1)));
    omega = omega(1:length(y(:,1)));
    amplitude = amplitude(1:length(y(:,1)));
    phase = phase(1:length(y(:,1)));
end