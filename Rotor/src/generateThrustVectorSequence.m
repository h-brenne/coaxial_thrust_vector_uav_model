function [omegas, amplitudes, phases] = generateThrustVectorSequence(min_omega, max_omega, step_omega, min_amplitude, max_amplitude, step_amplitude, min_phase, max_phase, step_phase, enable_dual_amplitude_steps)
    % Based on 
    % github.com/h-brenne/thrust_vector_control/blob/main/src/controller/thrust_vector_sequence_generator.cpp
    % Preallocate memory for output vectors
    numDataPoints = ceil(((max_omega - min_omega) / step_omega + 1) * ((max_amplitude - min_amplitude) / step_amplitude + 1) * ((max_phase - min_phase) / step_phase + 1) * (1 + enable_dual_amplitude_steps));
    omegas = zeros(1, numDataPoints);
    amplitudes = zeros(1, numDataPoints);
    phases = zeros(1, numDataPoints);
    
    % Initialization of vector index
    idx = 1;

    for omega = min_omega : step_omega : max_omega
        for amplitude = min_amplitude : step_amplitude : max_amplitude
            for phase = min_phase : step_phase : max_phase
                omegas(idx) = omega;
                amplitudes(idx) = amplitude;
                phases(idx) = phase;
                idx = idx + 1;  % Update index
            end
        end

        % Move amplitude back to minimum in steps. This is to reduce stresses during calibration
        if enable_dual_amplitude_steps
            for amplitude = max_amplitude : -step_amplitude : min_amplitude
                for phase = min_phase : step_phase : max_phase
                    omegas(idx) = omega;
                    amplitudes(idx) = amplitude;
                    phases(idx) = phase;
                    idx = idx + 1;  % Update index
                end
            end
        end
    end

    % Trim excess zeros from preallocated arrays
    omegas = omegas(1:idx-1);
    amplitudes = amplitudes(1:idx-1);
    phases = phases(1:idx-1);

end
