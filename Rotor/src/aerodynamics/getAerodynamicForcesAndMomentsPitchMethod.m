function [f_ext, OpRot_positive, OpRot_negative] = getAerodynamicForcesAndMomentsPitchMethod(BEMT_config, rotor, q, q_d)
%getAerodynamicMoments Get aerodynamic forces and moments in base frame

% Positive side hub
OpRot = BEMT_config.OpRot;
rpm = convangvel(q_d(1), 'rad/s', 'rpm');


[wrench_positive, OpRot_positive] = calculateWrench('side_hub_positive_1', rpm, 1);

%% Negative side hub

[wrench_negative, OpRot_negative] = calculateWrench('side_hub_negative_1', rpm, -1);

f_ext_1 = externalForce(rotor,'side_hub_positive_1',wrench_positive, q);
f_ext_2 = externalForce(rotor,'side_hub_negative_1',wrench_negative, q);
% Can f_ext be summed like this?
f_ext = f_ext_1 + f_ext_2;

    function [wrench, OpRot_result] = calculateWrench(side_hub, rpm, multiplier)
        % Get the transformation from 'teetering_hub_1' to 'side_hub'
        T = getTransform(rotor, q, side_hub, 'teetering_hub_1');
        
        % Extract the rotation matrix
        R = T(1:3,1:3);
        
        % The columns of R represent the x, y, z directions of 'side_hub_positive_1'
        % frame in 'teetering_hub_1' frame coordinates.
        y_dir = R(:,2);
        
        % Compute pitch angle between y_dir and the yz-plane in
        % 'teetering_hub_1' frame.
        yz_plane_normal = cross([0; 1; 0], [0; 0; 1]); % normal of the yz-plane in 'teetering_hub_1' frame
        pitch = multiplier * atan2(dot(y_dir, yz_plane_normal), dot(y_dir, [0; 1; 0]));
        
        Op = Oper(OpRot.Op.alt, OpRot.Op.speed, rpm, rad2deg(OpRot.Op.coll)+rad2deg(pitch), OpRot.Op.Flow.fluid);
        OpRot_result = OperRotor(OpRot.Rot, Op);
        bemt(OpRot_result, BEMT_config.Mod);
        dT = OpRot_result(1,1).ElPerf.dT;
        dQ = OpRot_result(1,1).ElPerf.dQ;
        dFz = dT;
        dFy = multiplier * dQ./OpRot.Rot.Bl.y;
           
        dF_teeter = [zeros(length(dFy), 1), dFy', dFz']; % force vectors in 'teetering_hub' frame
        dF_side = (R' * dF_teeter.').';  % force vectors in 'side_hub' frame
        r = -multiplier*linspace(OpRot.Rot.Bl.y(1)-BEMT_config.lag_pitch_hinge_offset, OpRot.Rot.Bl.y(end)-BEMT_config.lag_pitch_hinge_offset, OpRot.Rot.Bl.nElem).';
        r_mat = [r, zeros(length(r), 2)];
        moments = cross(r_mat, dF_side, 2);  % compute the moments due to each force
        total_moment = trapz(moments);  % integrate to get total moment
        total_force = sum(dF_side);  % sum the forces

        wrench = [total_moment, total_force];
    end
end
