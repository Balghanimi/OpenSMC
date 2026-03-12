classdef GlobalSurface < core.SlidingSurface
    %GLOBALSURFACE Global sliding surface (zero reaching phase).
    %
    %   s(t) = edot + c*e - (edot(0) + c*e(0)) * exp(-alpha*t)
    %
    %   By construction s(0) = 0, so the system starts ON the sliding
    %   surface from t=0. This eliminates the reaching phase entirely,
    %   providing robustness from the very first instant.
    %
    %   The exponential decay term vanishes as t -> inf, recovering
    %   the classical linear sliding surface s = edot + c*e.
    %
    %   Parameters:
    %       c     - surface slope (default 10)
    %       alpha - decay rate of initial offset (default 5)
    %
    %   Reference:
    %       Bartoszewicz, A. (1998). "Discrete-time quasi-sliding-mode
    %       control strategies." IEEE TIE, 45(4), 633-637.
    %       Also: Zhihong, M. & Yu, X.H. (1999). "Global sliding mode
    %       control." IJOC, 72(12), 1096-1109.

    properties
        name = 'Global'
    end

    methods
        function obj = GlobalSurface(varargin)
            p.c     = 10;
            p.alpha = 5;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('s0', [], 't0', []);
        end

        function s = compute(obj, e, edot, ~, t)
            % Compute the nominal surface value
            s_nominal = edot + obj.params.c * e;

            % Capture initial offset on first call
            if isempty(obj.state.s0)
                obj.state.s0 = s_nominal;
                obj.state.t0 = t;
            end

            % Subtract decaying initial offset so s(0) = 0
            dt = t - obj.state.t0;
            s = s_nominal - obj.state.s0 .* exp(-obj.params.alpha * dt);
        end

        function reset(obj)
            obj.state = struct('s0', [], 't0', []);
        end
    end
end
