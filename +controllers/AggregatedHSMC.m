classdef AggregatedHSMC < core.Controller
    %AGGREGATEDHSMC Aggregated hierarchical sliding mode controller.
    %
    %   Decomposes a 4-state underactuated system into two subsystems and
    %   constructs a two-layer hierarchical sliding surface:
    %
    %   Layer 1 (subsystem surfaces):
    %       s1 = c1 * e1 + e2       (trolley: position + velocity)
    %       s2 = c2 * e3 + e4       (payload: angle + angular velocity)
    %
    %   Layer 2 (aggregated surface):
    %       S = alpha * s1 + s2
    %
    %   Control law (Eq. 4.15):
    %       u = (alpha*b1*ueq1 + b2*ueq2 - kappa*S - eta*sgn(S))
    %           / (alpha*b1 + b2)
    %
    %   where:
    %       ueq1 = -(c1*x2 + f1(x)) / b1(x)
    %       ueq2 = -(c2*x4 + f2(x)) / b2(x)
    %       f_i, b_i are from the plant's dynamics decomposition.
    %
    %   Parameters:
    %       c1    - (scalar) trolley subsystem slope (default 0.7)
    %       c2    - (scalar) payload subsystem slope (default 8.2)
    %       alpha - (scalar) aggregation weight (default -2.3)
    %       kappa - (scalar) sliding gain (default 3)
    %       eta   - (scalar) switching gain (default 0.1)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Sec 4.2, Springer.

    properties
        name = 'AggregatedHSMC'
    end

    methods
        function obj = AggregatedHSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.c1    = 0.7;
            p.c2    = 8.2;
            p.alpha = -2.3;
            p.kappa = 3;
            p.eta   = 0.1;
            obj.params = p;
            obj.state  = struct();
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p = obj.params;

            % Error: assumes 4-state plant [x1, x1dot, x3, x3dot]
            e = xref - x;
            e1 = e(1); e2 = e(2); e3 = e(3); e4 = e(4);

            % First-layer sliding surfaces
            s1 = p.c1 * e1 + e2;
            s2 = p.c2 * e3 + e4;

            % Second-layer aggregated surface
            S = p.alpha * s1 + s2;

            % Get plant dynamics decomposition
            [f1, f2, b1, b2] = plant.get_dynamics_components(x);

            % Equivalent controls for each subsystem (Eq. 4.7-4.8)
            ueq1 = -(p.c1 * x(2) + f1) / b1;
            ueq2 = -(p.c2 * x(4) + f2) / b2;

            % Switching control (Eq. 4.9 / 4.15)
            den = p.alpha * b1 + b2;
            num = p.alpha * b1 * ueq1 + b2 * ueq2 ...
                  + p.kappa * S + p.eta * sign(S);
            u = num / den;

            % Estimate disturbance (optional compensation)
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, u, y);
            u = u - dhat(1:plant.n_inputs);

            % Diagnostics
            info.s     = S;       % main surface (for benchmark logging)
            info.s1    = s1;
            info.s2    = s2;
            info.S     = S;
            info.ueq1  = ueq1;
            info.ueq2  = ueq2;
            info.dhat  = dhat;
            info.e     = e;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct();
        end
    end
end
