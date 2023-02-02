classdef CoaxialRotorcraftModel
    % State: [n, e, d, u, v, w, p, q, r, phi, theta, psi]
    % Input: [F_x F_y F_z M_x M_y M_z]
    % Structure based on https://github.com/bernhardpg/babyshark_vtol_model
    properties
        input_function;
    end
    properties(Constant)
        g = 9.81;
        
        mass = 1.5; % kg
        I_xx = 1;
        I_yy = 1;
        I_zz = 1;
        
        % Upper and lower rotor horizontal displacement from CoM
        r_u = 0.15;
        r_l = -0.15;
        
        % Max forces and moments
        max_angle = deg2rad(30);
        max_force_magnitude = 20; % N
        F_x_max = CoaxialRotorcraftModel.max_force_magnitude * sin(CoaxialRotorcraftModel.max_angle);
        F_y_max = CoaxialRotorcraftModel.max_force_magnitude * sin(CoaxialRotorcraftModel.max_angle);
        F_z_max = CoaxialRotorcraftModel.max_force_magnitude;
        M_x_max = CoaxialRotorcraftModel.F_x_max*CoaxialRotorcraftModel.r_u;
        M_y_max = CoaxialRotorcraftModel.F_y_max*CoaxialRotorcraftModel.r_u;
        M_z_max = 10;
        
    end
    
    methods
        function obj = CoaxialRotorcraftModel(input_function)
            obj.input_function = input_function;
        end
        
        function x_dot = f(obj, t, x)
            % Unpack states
            x_cell = num2cell(x);
            [n, e, d, u, v, w, p, q, r, phi, theta, psi] = x_cell{:};
            % Unpack inputs
            input = obj.input_function(t, x);
            input_cell = num2cell(input);
            
    
            % TODO: Actuator simulation
            [F_x, F_y, F_z, M_x, M_y, M_z] = input_cell{:};
            
            % Constrain input
            [F_x, F_y, F_z, M_x, M_y, M_z] = obj.bound_input(F_x, F_y, F_z, M_x, M_y, M_z);
            
            % Kinematics
            R = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(psi)*sin(theta);
                 cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                 -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
            pos_dot = R * [u; v; w];
            n_dot = pos_dot(1);
            e_dot = pos_dot(2);
            d_dot = pos_dot(3);
            
            phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi));
            theta_dot = q * cos(phi) - r * sin(phi);
            psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);
            
            
            % Dynamics
            % Gravity
            gravity_vector_body = R'*[0 0 obj.g]';
            F_x = F_x + gravity_vector_body(1);
            F_y = F_y + gravity_vector_body(2);
            F_z = F_z + gravity_vector_body(3);
            % Full nonlinear body dynamics with diagonal inertia matrix
            u_dot = 1/obj.mass*F_x - q*w - r*w;
            v_dot = 1/obj.mass*F_y + p*w - r*u;
            w_dot = 1/obj.mass*F_z - p*v + q*u;
            
            p_dot = (1/obj.I_xx)*(M_x - (obj.I_zz - obj.I_yy)*q*r);
            q_dot = (1/obj.I_yy)*(M_y + (obj.I_xx - obj.I_zz)*p*r);
            r_dot = (1/obj.I_zz)*(M_z - (obj.I_yy - obj.I_xx)*p*q);
            
            x_dot = [n_dot e_dot d_dot u_dot v_dot w_dot p_dot q_dot r_dot phi_dot theta_dot psi_dot]';
        end
        function [F_x, F_y, F_z, M_x, M_y, M_z] = bound_input(obj, F_x, F_y, F_z, M_x, M_y, M_z)
            if abs(F_x) >= obj.F_x_max
                F_x = sign(F_x)*obj.F_x_max;
            end
            if abs(F_y) >= obj.F_y_max
                F_y = sign(F_y)*obj.F_y_max;
            end
            if abs(F_z) >= obj.F_z_max
                F_z = sign(F_z)*obj.F_z_max;
            end
            if abs(M_x) >= obj.M_x_max
                M_x = sign(M_x)*obj.M_x_max;
            end
            if abs(M_y) >= obj.M_y_max
                M_y = sign(M_y)*obj.M_y_max;
            end
            if abs(M_z) >= obj.M_z_max
                M_z = sign(M_z)*obj.M_z_max;
            end
        end
    end
end