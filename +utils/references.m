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
    end
end

function xref = multi_step_eval(t, targets, switch_times, n_states)
    idx = find(t >= switch_times, 1, 'last');
    if isempty(idx), idx = 1; end
    target = targets(idx);
    xref = [target; zeros(n_states - 1, 1)];
end
