classdef LinearSurface < core.SlidingSurface
    %LINEARSURFACE Classical linear sliding surface.
    %
    %   s = edot + c * e
    %
    %   The simplest and most common surface. Guarantees asymptotic
    %   convergence to the origin once sliding mode is established.
    %   Does NOT guarantee finite-time convergence (see TerminalSurface).
    %
    %   Reference: Utkin, V.I. (1977). "Variable structure systems
    %   with sliding modes." IEEE TAC, 22(2), 212-222.

    properties
        name = 'Linear'
    end

    methods
        function obj = LinearSurface(varargin)
            p.c = 10;  % default slope
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            s = edot + obj.params.c * e;
        end
    end
end
