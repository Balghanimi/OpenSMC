classdef CombiningHSMC < core.Controller
    %COMBININGHSMC Combining hierarchical sliding mode controller.
    %
    %   Uses derivative relations between states to define an intermediate
    %   variable and constructs the sliding surface from it:
    %
    %   Intermediate variable:
    %       z = x1 + c * x3
    %
    %   Sliding surface (using z and its derivative):
    %       s = alpha * z + z_dot
    %         = alpha * (x1 + c*x3) + (x2 + c*x4)
    %
    %   Control law (Eq. 4.45-4.46, 4.50):
    %       ueq = -(c*f2 + f1 + alpha*c*x4 + alpha*x2) / (c*b2 + b1)
    %       usw = -kappa*s - eta*sgn(s)
    %       u   = ueq + usw / (c*b2 + b1)
    %
    %   Sign-switching for asymptotic stability (Eq. 4.58):
    %       c = |c|  if x1*x3 >= 0
    %       c = -|c| if x1*x3 < 0
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

            % Sign-switching rule for c (Eq. 4.58)
            if x(1) * x(3) >= 0
                c_eff = abs(p.c);
            else
                c_eff = -abs(p.c);
            end

            % Intermediate variable z = x1 + c*x3
            z    = x(1) + c_eff * x(3);
            zdot = x(2) + c_eff * x(4);

            % Combining sliding surface: s = alpha*z + zdot
            s = p.alpha * z + zdot;

            % Get plant dynamics decomposition
            [f1, f2, b1, b2] = plant.get_dynamics_components(x);

            % Equivalent control (Eq. 4.46)
            den = c_eff * b2 + b1;
            ueq = -(c_eff * f2 + f1 + p.alpha * c_eff * x(4) + p.alpha * x(2)) / den;

            % Switching control (Eq. 4.50)
            ds = -p.kappa * s - p.eta * sign(s);

            % Total control
            u = ueq + ds / den;

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
            info.e    = xref - x;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct();
        end
    end
end
