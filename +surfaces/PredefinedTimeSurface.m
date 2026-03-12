classdef PredefinedTimeSurface < core.SlidingSurface
    %PREDEFINEDTIMESURFACE Predefined-time convergent sliding surface.
    %
    %   s = edot + (pi / (2*Tc)) * e ./ cos((pi/2) * (t/Tc))
    %
    %   for t < Tc, and s = edot + c_inf * e  for t >= Tc.
    %
    %   This surface guarantees e(t) -> 0 before the user-specified
    %   convergence time Tc, regardless of initial conditions or
    %   disturbance bounds. The time-varying gain increases as t -> Tc,
    %   forcing convergence before the deadline.
    %
    %   After Tc, the surface reverts to a standard linear surface
    %   to maintain sliding mode.
    %
    %   Parameters:
    %       Tc    - predefined convergence time [s] (default 2.0)
    %       c_inf - post-convergence slope (default 10)
    %
    %   Reference:
    %       Sanchez-Torres, J.D. et al. (2018). "Predefined-time
    %       stability of dynamical systems with sliding modes."
    %       Mathematical Problems in Engineering, 2018.

    properties
        name = 'PredefinedTime'
    end

    methods
        function obj = PredefinedTimeSurface(varargin)
            p.Tc    = 2.0;
            p.c_inf = 10;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, t)
            Tc = obj.params.Tc;

            if t < Tc
                % Time-varying gain that goes to infinity as t -> Tc
                ratio = t / Tc;
                % Clamp to avoid numerical issues near Tc
                ratio = min(ratio, 0.999);
                c_t = (pi / (2 * Tc)) ./ cos((pi/2) * ratio);
                s = edot + c_t .* e;
            else
                % After predefined time, use standard linear surface
                s = edot + obj.params.c_inf .* e;
            end
        end
    end
end
