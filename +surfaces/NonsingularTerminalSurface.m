classdef NonsingularTerminalSurface < core.SlidingSurface
    %NONSINGULARTERMINALSURFACE Nonsingular fast terminal sliding surface.
    %
    %   s = e + (1/beta) * |edot|^(q/p) * sign(edot)
    %
    %   Resolves the singularity problem of TerminalSurface by reformulating
    %   so that the fractional power acts on edot instead of e.
    %   Guarantees finite-time convergence WITHOUT singularity.
    %
    %   For FAST version (adds linear term for faster far-from-origin convergence):
    %   s = e + (1/beta) * |edot|^(q/p) * sign(edot) + alpha * edot
    %
    %   Reference: Yu, X., & Zhihong, M. (2002). "Fast terminal
    %   sliding-mode control design for nonlinear dynamical systems."
    %   IEEE Trans. Circuits Syst. I, 49(2), 261-264.

    properties
        name = 'NonsingularTerminal'
    end

    methods
        function obj = NonsingularTerminalSurface(varargin)
            p.alpha = 0;     % linear term (0 = standard, >0 = fast version)
            p.beta  = 10;
            p.p     = 7;     % numerator (odd, p > q for nonsingular)
            p.q     = 5;     % denominator (odd)
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            qp = obj.params.q / obj.params.p;
            s = e + (1/obj.params.beta) * abs(edot).^qp .* sign(edot);
            if obj.params.alpha > 0
                s = s + obj.params.alpha * edot;  % fast version
            end
        end
    end
end
