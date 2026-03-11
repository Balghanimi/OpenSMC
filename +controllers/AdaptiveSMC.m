classdef AdaptiveSMC < core.Controller
    %ADAPTIVESMC Adaptive sliding mode controller with online gain tuning.
    %
    %   Adapts the switching gain online using a Lyapunov-based law,
    %   reducing conservatism from fixed over-estimated gains.
    %
    %   Adaptive law (with projection):
    %       theta_dot = -gamma * edot * s
    %
    %   with projection to keep theta in [theta_min, theta_max].
    %   The adapted parameter theta modulates the equivalent control:
    %
    %       u = theta * phi(x) - (eta + eta_0) * sign(s)
    %
    %   where phi(x) is a known regressor (e.g., edot for simple plants).
    %
    %   Parameters:
    %       gamma     - (scalar) adaptation rate (default 500)
    %       eta       - (scalar) base switching gain (default 0.5)
    %       theta_min - (scalar) lower bound on theta (default -50)
    %       theta_max - (scalar) upper bound on theta (default 50)
    %       c         - (scalar) surface slope (default 15)
    %       delta     - (scalar) boundary layer width, 0=sgn (default 0)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 6, Springer.

    properties
        name = 'AdaptiveSMC'
    end

    methods
        function obj = AdaptiveSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.gamma     = 500;    % adaptation rate
            p.eta       = 0.5;    % base switching gain
            p.theta_min = -50;    % projection lower bound
            p.theta_max = 50;     % projection upper bound
            p.delta     = 0;      % boundary layer (0 = pure sign)
            p.dt        = 1e-4;   % integration step
            obj.params = p;
            obj.state  = struct('eint', 0, 'theta', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            % Get error
            [e, edot] = plant.get_error(x, xref);

            % Integral of error
            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end

            % Compute sliding variable
            s = obj.surface.compute(e, edot, obj.state.eint, t);

            % Estimate disturbance
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, zeros(plant.n_inputs, 1), y);

            % Adaptive law with projection (Eq. 6.1)
            theta = obj.state.theta;
            gamma = obj.params.gamma;
            dt_   = obj.params.dt;

            % Adaptation: theta_dot = -gamma * edot * s
            alaw = -gamma * edot(1) * s(1);

            % Projection
            if (theta >= obj.params.theta_max && alaw > 0) || ...
               (theta <= obj.params.theta_min && alaw < 0)
                alaw = 0;
            end
            theta = theta + dt_ * alaw;
            theta = max(obj.params.theta_min, min(obj.params.theta_max, theta));
            obj.state.theta = theta;

            % Switching function
            if obj.params.delta > 0
                sat_s = min(1, max(-1, s / obj.params.delta));
            else
                sat_s = sign(s);
            end

            % Reaching control
            u_r = obj.reaching.compute(s, t);

            % Adaptive control law
            % u = theta * phi(x) + reaching + dhat compensation
            u_adaptive = theta * edot(1:plant.n_inputs);
            u = u_adaptive - u_r - dhat(1:plant.n_inputs);

            % Diagnostics
            info.s     = s;
            info.ueq   = u_adaptive;
            info.ur    = u_r;
            info.dhat  = dhat;
            info.theta = theta;
            info.e     = e;
            info.edot  = edot;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('eint', 0, 'theta', 0);
        end
    end
end
