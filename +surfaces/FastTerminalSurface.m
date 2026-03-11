classdef FastTerminalSurface < core.SlidingSurface
    %FASTTERMINALSURFACE Global fast terminal sliding surface.
    %
    %   s = edot + alpha * e + beta * |e|^(q/p) * sign(e)
    %
    %   Combines linear convergence (alpha*e, fast far from origin) with
    %   finite-time convergence (beta*|e|^(q/p)*sign(e), near origin).
    %   Achieves GLOBAL finite-time convergence without singularity.
    %
    %   Parameters:
    %       alpha - (scalar) linear coefficient (default 2)
    %       beta  - (scalar) terminal coefficient (default 1)
    %       p     - (int, odd) numerator of power ratio (default 9)
    %       q     - (int, odd) denominator, q < p (default 5)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 7.3, Springer.

    properties
        name = 'FastTerminal'
    end

    methods
        function obj = FastTerminalSurface(varargin)
            p.alpha = 2;    % linear coefficient
            p.beta  = 1;    % terminal coefficient
            p.p     = 9;    % numerator (odd)
            p.q     = 5;    % denominator (odd, q < p)
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            qp = obj.params.q / obj.params.p;
            s = edot + obj.params.alpha * e ...
                + obj.params.beta * abs(e).^qp .* sign(e);
        end
    end
end
