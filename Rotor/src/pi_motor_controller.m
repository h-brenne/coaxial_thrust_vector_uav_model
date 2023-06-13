function [motor_torque] = pi_motor_controller(speed, e, speed_setpoint)
    
    % P controller for now
    Kp = 0.1;
    Ki = 7.0;
    error = speed_setpoint - speed;
    motor_torque = Kp*error + Ki*e;
end