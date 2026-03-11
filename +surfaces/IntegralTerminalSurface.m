classdef IntegralTerminalSurface < core.SlidingSurface
    %INTEGRALTERMINALSURFACE Integral terminal sliding surface (ITSMC).
    %
    %   s = edot + c1 * e + c2 * integral(|e|^(p/q) * sign(e)) dt
    %
    %   Combines integral action (for zero steady-state error and
    %   robustness from t=0) with terminal attractor (for finite-time
    %   convergence). Eliminates the reaching phase entirely.
    %
    %   This is the surface used in Ali's master student's ITSMC-ELM work.

    properties
        name = 'IntegralTerminal'
    end

    methods
        function obj = IntegralTerminalSurface(varargin)
            p.c1 = 10;
            p.c2 = 5;
            p.p  = 5;    % odd
            p.q  = 7;    % odd, p < q
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, eint, ~)
            pq = obj.params.p / obj.params.q;
            % eint is the integral of |e|^(p/q)*sign(e), computed externally
            % If raw integral of e is passed, apply the nonlinear map here
            frac_int = abs(eint).^pq .* sign(eint);
            s = edot + obj.params.c1 * e + obj.params.c2 * frac_int;
        end
    end
end
