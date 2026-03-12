classdef CombiningHSMC < core.Controller
    %COMBININGHSMC Combining hierarchical sliding mode controller.
    %
    %   Uses derivative relations between error states to define an
    %   intermediate variable and constructs the sliding surface from it:
    %
    %   Intermediate variable (error-based for tracking):
    %       z = e1 + c * e3     where e = xref - x
    %
    %   Sliding surface (using z and its derivative):
    %       s = alpha * z + z_dot
    %         = alpha * (e1 + c*e3) + (e2 + c*e4)
    %
    %   Control law (derived from error dynamics):
    %       ueq = (alpha*(e2+c*e4) - (f1+c*f2)) / (b1+c*b2)
    %       usw = (kappa*s + eta*sgn(s)) / (b1+c*b2)
    %       u   = ueq + usw
    %
    %   Sign-switching for asymptotic stability (Eq. 4.58):
    %       c = |c|  if e1*e3 >= 0
    %       c = -|c| if e1*e3 < 0
    %
    %   Parameters:
    %       c     - (scalar) combining coefficient magnitude (default 0.242)
    %       alpha - (scalar) surface slope (default 0.487)
    %       kappa - (scalar) sliding gain (default 4)
    %       eta   - (scalar) switching gain (default 0.1)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Sec 4.4, Springer.

    properties
        name = 'CombiningHSMC'
    end

    methods
        function obj = CombiningHSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.c     = 0.242;
            p.alpha = 0.487;
            p.kappa = 4;
            p.eta   = 0.1;
            obj.params = p;
            obj.state  = struct();
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p = obj.params;

            % Error-based formulation for tracking
            e = xref - x;

            % Sign-switching rule for c (Eq. 4.58, applied to error)
            if e(1) * e(3) >= 0
                c_eff = abs(p.c);
            else
                c_eff = -abs(p.c);
            end

            % Intermediate variable z = e1 + c*e3
            z    = e(1) + c_eff * e(3);
            zdot = e(2) + c_eff * e(4);

            % Combining sliding surface: s = alpha*z + zdot
            s = p.alpha * z + zdot;

            % Get plant dynamics decomposition
            [f1, f2, b1, b2] = plant.get_dynamics_components(x);

            % Equivalent control (error-based derivation)
            % sdot = alpha*(e2+c*e4) - (f1+c*f2) - (b1+c*b2)*u
            den = b1 + c_eff * b2;
            ueq = (p.alpha * (e(2) + c_eff * e(4)) - (f1 + c_eff * f2)) / den;

            % Switching control (drives s → 0)
            usw = (p.kappa * s + p.eta * sign(s)) / den;

            % Total control
            u = ueq + usw;

            % Estimate disturbance (optional compensation)
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, u, y);
            u = u - dhat(1:plant.n_inputs);

            % Diagnostics
            info.z    = z;
            info.zdot = zdot;
            info.s    = s;
            info.c    = c_eff;
            info.ueq  = ueq;
            info.dhat = dhat;
            info.e    = e;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct();
        end
    end
end
