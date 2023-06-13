function [omega_tilde] = sinusoidal_control_law(motor_position, omega, amplitude, phase)
%INPUT_GENERATOR Generate the three control inputs for the motor
% Motor speed setpoint is omega + omega_tilde
omega_tilde = omega*amplitude*sin(motor_position + phase);
end
