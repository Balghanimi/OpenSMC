classdef FixedTimeSMC < core.Controller
    %FIXEDTIMESMC Fixed-time sliding mode controller.
    %
    %   Guarantees convergence to the sliding surface in a FIXED time
    %   T_max that is bounded regardless of initial conditions.
    %
    %   Reaching law (bi-power):
    %       sdot = -alpha * |s|^p * sign(s) - beta * |s|^q * sign(s)
    %
    %   with 0 < p < 1 < q. The settling time is bounded by:
    %       T_max <= 1 / (alpha*(1-p)) + 1 / (beta*(q-1))
    %
    %   Control law:
    %       u = -alpha * |s|^p * sign(s) - beta * |s|^q * sign(s) - dhat
    %
    %   The p < 1 term dominates near s=0 (fast local convergence).
    %   The q > 1 term dominates far from s=0 (fast global convergence).
    %   Together they give fixed-time convergence.
    %
    %   Parameters:
    %       alpha - gain for sub-linear term (default 10)
    %       beta  - gain for super-linear term (default 10)
    %       p     - sub-linear exponent, 0 < p < 1 (default 0.5)
    %       q     - super-linear exponent, q > 1 (default 1.5)
    %
    %   Reference:
    %       Polyakov, A. (2012). "Nonlinear feedback design for fixed-time
    %       stabilization of linear control systems." IEEE TAC, 57(8),
    %       2106-2110.

    properties
        name = 'FixedTimeSMC'
    end

    methods
        function obj = FixedTimeSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.alpha = 10;
            p.beta  = 10;
            p.p     = 0.5;    % 0 < p < 1
            p.q     = 1.5;    % q > 1
            obj.params = p;
            obj.state  = struct('eint', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            [e, edot] = plant.get_error(x, xref);

            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end

            % Compute sliding variable
            s = obj.surface.compute(e, edot, obj.state.eint, t);

            % Fixed-time bi-power reaching law
            alpha_ = obj.params.alpha;
            beta_  = obj.params.beta;
            p_exp  = obj.params.p;
            q_exp  = obj.params.q;

            u_r = alpha_ * abs(s).^p_exp .* sign(s) ...
                + beta_  * abs(s).^q_exp .* sign(s);

            % Disturbance estimate
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, zeros(plant.n_inputs,1), y);

            % Total control
            % u_r already points toward the surface (same sign as s),
            % so the corrective action is u = +u_r (not negated)
            u = u_r - dhat(1:plant.n_inputs);

            % Compute theoretical T_max
            T_max = 1/(alpha_*(1-p_exp)) + 1/(beta_*(q_exp-1));

            info.s     = s;
            info.ueq   = 0;
            info.ur    = u_r;
            info.dhat  = dhat;
            info.e     = e;
            info.edot  = edot;
            info.T_max = T_max;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('eint', 0);
        end

        function T = get_max_settling_time(obj)
            %GET_MAX_SETTLING_TIME Theoretical upper bound on convergence time.
            T = 1/(obj.params.alpha*(1-obj.params.p)) ...
              + 1/(obj.params.beta*(obj.params.q-1));
        end
    end
end
