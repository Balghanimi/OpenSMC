classdef IncrementalHSMC < core.Controller
    %INCREMENTALHSMC Incremental hierarchical sliding mode controller.
    %
    %   Builds a three-layer hierarchical structure by incrementally
    %   adding state variables to the sliding surfaces:
    %
    %   Layer 1: s1 = c1 * x1 + x2
    %   Layer 2: s2 = c2 * x3 + s1      (sign of c2 switches per Eq. 4.34)
    %   Layer 3: s3 = c3 * x4 + s2
    %
    %   The key feature is the sign-switching rule for c2 (Eq. 4.34):
    %       c2 = |c2|  if x3 * s1 >= 0
    %       c2 = -|c2| if x3 * s1 < 0
    %   This guarantees stability of all three layers.
    %
    %   Control law (Eq. 4.27, 4.31-4.32):
    %       ueq = -(c3*f2 + f1 + c2*x4 + c1*x2) / (c3*b2 + b1)
    %       usw = -(kappa*s3 + eta*sgn(s3)) / (c3*b2 + b1)
    %       u   = ueq + usw
    %
    %   Parameters:
    %       c1    - (scalar) layer 1 slope (default 0.85)
    %       c2    - (scalar) layer 2 slope magnitude (default 3.6)
    %       c3    - (scalar) layer 3 slope (default 0.4)
    %       kappa - (scalar) sliding gain (default 3)
    %       eta   - (scalar) switching gain (default 0.1)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Sec 4.3, Springer.

    properties
        name = 'IncrementalHSMC'
    end

    methods
        function obj = IncrementalHSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.c1    = 0.85;
            p.c2    = 3.6;
            p.c3    = 0.4;
            p.kappa = 3;
            p.eta   = 0.1;
            obj.params = p;
            obj.state  = struct();
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p = obj.params;

            % State error
            e = xref - x;

            % Layer 1: s1 = c1 * e1 + e2
            s1 = p.c1 * e(1) + e(2);

            % Sign-switching rule for c2 (Eq. 4.34)
            if x(3) * s1 >= 0
                c2_eff = abs(p.c2);
            else
                c2_eff = -abs(p.c2);
            end

            % Layer 2: s2 = c2 * e3 + s1
            s2 = c2_eff * e(3) + s1;

            % Layer 3: s3 = c3 * e4 + s2
            s3 = p.c3 * e(4) + s2;

            % Get plant dynamics decomposition
            [f1, f2, b1, b2] = plant.get_dynamics_components(x);

            % Equivalent control (Eq. 4.31)
            den = p.c3 * b2 + b1;
            ueq = -(p.c3 * f2 + f1 + c2_eff * x(4) + p.c1 * x(2)) / den;

            % Switching control (Eq. 4.32)
            usw = (p.kappa * s3 + p.eta * sign(s3)) / den;

            % Total control
            u = ueq + usw;

            % Estimate disturbance (optional compensation)
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, u, y);
            u = u - dhat(1:plant.n_inputs);

            % Diagnostics
            info.s    = s3;      % top-level surface (for benchmark logging)
            info.s1   = s1;
            info.s2   = s2;
            info.s3   = s3;
            info.ueq  = ueq;
            info.usw  = usw;
            info.c2   = c2_eff;
            info.dhat = dhat;
            info.e    = e;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct();
        end
    end
end
