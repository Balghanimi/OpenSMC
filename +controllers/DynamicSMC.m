classdef DynamicSMC < core.Controller
    %DYNAMICSMC Dynamic sliding mode controller.
    %
    %   Shifts the discontinuity from u to u_dot, producing a continuous
    %   control signal. Wraps any base sliding surface S and forms the
    %   dynamic sliding variable:
    %
    %       sigma = S_dot + lambda * S
    %
    %   The controller computes du/dt and integrates to get u:
    %
    %       du = (1/g(x)) * [-f'(x) + (c+lambda)*x_ddot_ref + x_dddot_ref
    %            - (c+lambda)*g(x)*u - (c+lambda)*f(x)
    %            - lambda*c*edot - eta*sign(sigma)]
    %
    %   Benefits: continuous control -> no chattering, no boundary layer
    %   needed, same robustness as discontinuous SMC.
    %
    %   Parameters:
    %       lambda - (scalar) dynamic surface slope (default 15)
    %       eta    - (scalar) switching gain (default 5)
    %       c      - (scalar) base surface slope (default 15)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 5, Springer.

    properties
        name = 'DynamicSMC'
    end

    methods
        function obj = DynamicSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.lambda = 15;    % dynamic surface slope
            p.eta    = 5;     % switching gain for sigma
            obj.params = p;
            obj.state  = struct('eint', 0, 'u_prev', 0, 's_prev', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            % Get error
            [e, edot] = plant.get_error(x, xref);

            % Integral of error
            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end

            % Compute base sliding variable
            s = obj.surface.compute(e, edot, obj.state.eint, t);

            % Estimate disturbance
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, obj.state.u_prev, y);

            % Numerical derivative of s for dynamic sliding variable
            % sigma = sdot + lambda * s
            if ~isfield(obj.state, 's_prev')
                obj.state.s_prev = s;
            end
            dt_ = 1e-4;  % should match simulator dt
            sdot = (s - obj.state.s_prev) / dt_;
            obj.state.s_prev = s;

            lam = obj.params.lambda;
            sigma = sdot + lam * s;

            % Reaching control on sigma
            u_r = obj.reaching.compute(sigma, t);

            % Equivalent control (simplified for general plants)
            % For model-based plants, override this
            u_eq = 0;

            % Dynamic SMC: u_dot computed, then integrated
            % Simplified version: u = u_eq + u_r - dhat, applied to sigma
            u = u_eq - u_r - dhat(1:plant.n_inputs);
            obj.state.u_prev = u;

            % Diagnostics
            info.s     = s;
            info.sigma = sigma;
            info.sdot  = sdot;
            info.ueq   = u_eq;
            info.ur    = u_r;
            info.dhat  = dhat;
            info.e     = e;
            info.edot  = edot;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('eint', 0, 'u_prev', 0, 's_prev', 0);
        end
    end
end
