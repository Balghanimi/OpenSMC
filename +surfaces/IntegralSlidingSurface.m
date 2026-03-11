classdef IntegralSlidingSurface < core.SlidingSurface
    %INTEGRALSLIDINGSURFACE Integral sliding mode surface.
    %
    %   Eliminates the reaching phase by embedding the initial conditions
    %   into the surface definition. The system starts on the surface at t=0.
    %
    %   s = C * (x - x0) - integral{ C * (A - B*K) * x } dt
    %
    %   Simplified form for error-based systems (Qian Appendix C):
    %   s = C * (e - E)
    %   where E_dot = (A - B*K) * e  (integral state)
    %
    %   The surface C and feedback gain K are design parameters. K is
    %   typically chosen via pole placement or LQR on the nominal system.
    %
    %   Parameters:
    %       C     - (1 x n) surface coefficients (default [0.5 1.7 -3 -1])
    %       K     - (1 x n) nominal feedback gain (default [1.2 1.7 -6 -2])
    %       A     - (n x n) system matrix (plant-specific)
    %       B     - (n x 1) input matrix (plant-specific)
    %       dt    - (scalar) integration step (default 1e-4)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Ch 3, Appendix C, Springer.
    %   Also: Utkin & Shi (1996), "Integral sliding mode in systems
    %   operating under uncertainty conditions."

    properties
        name = 'IntegralSliding'
    end

    methods
        function obj = IntegralSlidingSurface(varargin)
            p.C  = [0.5, 1.7, -3, -1];
            p.K  = [1.2, 1.7, -6, -2];
            p.A  = [];   % must be set by user
            p.B  = [];   % must be set by user
            p.dt = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('E', []);
        end

        function s = compute(obj, e, edot, ~, ~)
            p = obj.params;
            n = numel(e);

            % Full error vector: [position_errors; velocity_errors]
            x_err = [e; edot];

            % Initialize integral state
            if isempty(obj.state.E) || numel(obj.state.E) ~= 2*n
                obj.state.E = zeros(2*n, 1);
            end

            % Compute integral dynamics: E_dot = (A - B*K) * x_err
            if ~isempty(p.A) && ~isempty(p.B)
                AB = p.A - p.B * p.K;
                E_dot = AB * x_err;
            else
                % Fallback: simple integral of error
                E_dot = x_err;
            end

            % Euler integration
            obj.state.E = obj.state.E + p.dt * E_dot;

            % Sliding variable: s = C * (x_err - E)
            C_vec = p.C;
            if numel(C_vec) ~= 2*n
                % Pad or truncate C to match state dimension
                C_full = zeros(1, 2*n);
                C_full(1:min(numel(C_vec), 2*n)) = C_vec(1:min(numel(C_vec), 2*n));
                C_vec = C_full;
            end

            s = C_vec * (x_err - obj.state.E);
            s = s(:);
        end

        function reset(obj)
            obj.state = struct('E', []);
        end
    end
end
