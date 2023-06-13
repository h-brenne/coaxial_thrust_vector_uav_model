function [f_ext, OpRot_positive, OpRot_negative] = getAerodynamicForcesAndMoments(BEMT_config, rotor, q, q_d, omega)
%getAerodynamicMoments Get aerodynamic forces and moments in base frame

% Positive side hub
OpRot = BEMT_config.OpRot;
% Average rpm, setpoint. Used for momentum theory
rpm = convangvel(omega, 'rad/s', 'rpm');

[wrench_positive, OpRot_positive] = calculateWrench('side_hub_positive_1', rpm, 1);
[wrench_negative, OpRot_negative] = calculateWrench('side_hub_negative_1', rpm, -1);

f_ext_1 = externalForce(rotor,'side_hub_positive_1',wrench_positive, q);
f_ext_2 = externalForce(rotor,'side_hub_negative_1',wrench_negative, q);
f_ext = f_ext_1 + f_ext_2;

    function [wrench, OpRot_result] = calculateWrench(side_hub, rpm, multiplier)
        J = geometricJacobian(rotor, q, side_hub);
        % Compute velocity in base frame
        v_global = J * q_d;
        % Get transform from base frame to side_hub frame
        H = getTransform(rotor, q, side_hub);
        % Extract the rotation matrix
        R = H(1:3,1:3);
        % Compute velocity in side_hub body frame
        v_body_angular = R.' * v_global(1:3);
        v_body_linear = R.' * v_global(4:6);
        
        % r : Distance of elements along x-axis
        % The total length is the radius of the propeller minus
        % root cutout. Config aligns root cutout and side_hub x-axis origin
        % side_hub_positive has flipped x-axis compared to
        % side_hub_negative
        r = -multiplier * linspace(OpRot.Rot.Bl.y(1)-BEMT_config.lag_pitch_hinge_offset, OpRot.Rot.Bl.y(end)-BEMT_config.lag_pitch_hinge_offset, OpRot.Rot.Bl.nElem).';
        r_mat = [r, zeros(length(r), 2)]; % Creating a matrix with each row as [r_i 0 0]
        cross_product_result = cross(repmat(v_body_angular.', length(r), 1), r_mat);
        
        % Find tangential and axial velocities along sections
        % For the positive hub, y-axis points opposite to nominal direction 
        y_velocities = -multiplier*(v_body_linear(2) + cross_product_result(:,2));
        z_velocities = (v_body_linear(3) + cross_product_result(:,3));
        
        Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, OpRot.Op.coll,...
            OpRot.Op.Flow.fluid, y_velocities, z_velocities);
        OpRot_result = OperRotor(OpRot.Rot, Op);
        %OpRot_result.ElPerf.tgSpeed = y_velocities;
        %OpRot_result.ElPerf.axSpeed = z_velocities;
        bemt(OpRot_result, BEMT_config.Mod);
        
        dT = OpRot_result(1,1).ElPerf.dT;
        dQ = OpRot_result(1,1).ElPerf.dQ;
        % dQ is multiplied by the radial position assumed by Rotare, which are
        % slightly different than ours due to hinge angles. Negative for 
        % side_hub_negative y-axis

        % side_hub frame differential forces
        dFz = dT;
        dFy = multiplier * dQ./OpRot.Rot.Bl.y;

        % Compute net force and moments
        Fy = trapz(dFy); % force along y-axis
        Fz = trapz(dFz); % force along z-axis
        
        Torque_y = -trapz(dFz.*r'); % moment about y-axis
        % Both produce negative torque_z due to r sign and y-direction
        Torque_z = trapz(dFy.*r'); % moment about z-axis

        % Compose the wrench
        wrench = [0; Torque_y; Torque_z; 0; Fy; Fz];
    end
end
