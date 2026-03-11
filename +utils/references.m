classdef references
    %REFERENCES Standard reference trajectory generators.
    %
    %   Each function returns a handle @(t) -> xref (full state reference).

    methods (Static)

        function fn = step_ref(target, n_states)
            %STEP_REF Step reference (position = target, velocity = 0).
            fn = @(t) [target; zeros(n_states - numel(target), 1)];
        end

        function fn = ramp(slope, n_states)
            %RAMP Linear ramp reference.
            fn = @(t) [slope * t; slope; zeros(n_states - 2, 1)];
        end

        function fn = sinusoidal(amplitude, frequency, n_states)
            %SINUSOIDAL Sinusoidal tracking reference.
            w = 2 * pi * frequency;
            fn = @(t) [amplitude * sin(w*t); ...
                        amplitude * w * cos(w*t); ...
                        zeros(n_states - 2, 1)];
        end

        function fn = multi_step(targets, switch_times, n_states)
            %MULTI_STEP Piecewise constant reference.
            fn = @(t) multi_step_eval(t, targets, switch_times, n_states);
        end

        function fn = transport(target_pos, n_states)
            %TRANSPORT Crane transport reference (position step, zero swing).
            %   target_pos: desired trolley position [m]
            %   n_states:   total state dimension (4 for single, 6 for double crane)
            xd = zeros(n_states, 1);
            xd(1) = target_pos;
            fn = @(t) xd;
        end

        function fn = circular_3d(radius, omega, z0, dz_amp, dz_freq)
            %CIRCULAR_3D 3D circular trajectory for quadrotor.
            %   radius:  circle radius [m]
            %   omega:   angular velocity [rad/s]
            %   z0:      base altitude [m]
            %   dz_amp:  altitude oscillation amplitude [m]
            %   dz_freq: altitude oscillation frequency [rad/s]
            %   Returns 12-state reference [x,y,z, 0,0,0, vx,vy,vz, 0,0,0]
            fn = @(t) circular_3d_eval(t, radius, omega, z0, dz_amp, dz_freq);
        end
    end
end

function xref = multi_step_eval(t, targets, switch_times, n_states)
    idx = find(t >= switch_times, 1, 'last');
    if isempty(idx), idx = 1; end
    target = targets(idx);
    xref = [target; zeros(n_states - 1, 1)];
end

function xref = circular_3d_eval(t, R, w, z0, dz_amp, dz_freq)
    % Smooth ramp-in to avoid aggressive initial transient
    ramp = 1 / (1 + exp(-3*(t - 2)));

    px = ramp * R * cos(w * t);
    py = ramp * R * sin(w * t);
    pz = ramp * (z0 + dz_amp * sin(dz_freq * t));

    % Velocity (approximate, ignoring ramp derivative for simplicity)
    vx = ramp * (-R * w * sin(w * t));
    vy = ramp * (R * w * cos(w * t));
    vz = ramp * (dz_amp * dz_freq * cos(dz_freq * t));

    xref = [px; py; pz; 0; 0; 0; vx; vy; vz; 0; 0; 0];
end
