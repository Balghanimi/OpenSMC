classdef TerminalSurface < core.SlidingSurface
    %TERMINALSURFACE Terminal sliding surface for finite-time convergence.
    %
    %   s = edot + beta * |e|^(p/q) * sign(e)
    %
    %   Guarantees finite-time convergence to origin. The fractional
    %   power p/q (with p < q, both odd) creates a terminal attractor.
    %
    %   WARNING: Singularity at e = 0, edot ~= 0. Use NonsingularTerminal
    %   for singularity-free design.
    %
    %   Reference: Zak, M. (1988). "Terminal attractors for addressable
    %   memory in neural networks." Physics Letters A, 133(1-2), 18-22.

    properties
        name = 'Terminal'
    end

    methods
        function obj = TerminalSurface(varargin)
            p.beta = 10;
            p.p    = 5;     % numerator (odd, p < q)
            p.q    = 7;     % denominator (odd)
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            pq = obj.params.p / obj.params.q;
            s = edot + obj.params.beta * abs(e).^pq .* sign(e);
        end
    end
end
