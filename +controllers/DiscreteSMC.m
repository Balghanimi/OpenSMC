classdef DiscreteSMC < core.Controller
    %DISCRETESMC Discrete-time sliding mode controller.
    %
    %   Uses the discrete reaching law (Gao, 1995):
    %
    %       s(k+1) = (1 - q*T) * s(k) - epsilon*T * sign(s(k))
    %
    %   which gives the control law:
    %
    %       u(k) = ueq(k) - (1/g) * [q*T*s(k) + epsilon*T*sign(s(k))]
    %
    %   where T is the sampling period, g is the input gain, and
    %   ueq is the discrete equivalent control.
    %
    %   This controller is designed for sampled-data / digital
    %   implementations where continuous-time sign switching is
    %   replaced by discrete reaching conditions.
    %
    %   Parameters:
    %       epsilon - switching gain (default 5)
    %       q       - exponential reaching rate (default 10)
    %       Ts      - sampling period [s] (default 1e-3)
    %       g       - input gain (default 1)
    %
    %   Reference:
    %       Gao, W. et al. (1995). "Discrete-time variable structure
    %       control systems." IEEE TIE, 42(2), 117-122.

    properties
        name = 'DiscreteSMC'
    end

    methods
        function obj = DiscreteSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.epsilon = 5;       % switching gain
            p.q       = 10;      % reaching rate
            p.Ts      = 1e-3;    % sampling period
            p.g       = 1;       % input gain
            obj.params = p;
            obj.state  = struct('eint', 0, 't_last', -inf, 'u_hold', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            [e, edot] = plant.get_error(x, xref);

            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end
            if ~isfield(obj.state, 'u_hold') || numel(obj.state.u_hold) ~= plant.n_inputs
                obj.state.u_hold = zeros(plant.n_inputs, 1);
            end

            % Zero-order hold: only update control at sampling instants
            if t - obj.state.t_last >= obj.params.Ts - 1e-10
                obj.state.t_last = t;

                % Compute sliding variable
                s = obj.surface.compute(e, edot, obj.state.eint, t);

                % Discrete reaching law control (Gao 1995)
                % Correct form: u(k) = (1/g) * [q*s(k) + epsilon*sign(s(k))]
                % This drives s(k+1) = (1-q*Ts)*s(k) - epsilon*Ts*sign(s(k))
                q  = obj.params.q;
                ep = obj.params.epsilon;
                g  = obj.params.g;

                u_r = (1/g) * (q*s + ep*sign(s));

                % Disturbance estimate
                y = plant.output(x);
                dhat = obj.estimator.estimate(t, x, obj.state.u_hold, y);

                % Total control (u_r already points toward surface)
                obj.state.u_hold = u_r - dhat(1:plant.n_inputs);
                obj.state.s_last = s;
            end

            u = obj.state.u_hold;

            % Diagnostics
            s_val = obj.surface.compute(e, edot, obj.state.eint, t);
            info.s    = s_val;
            info.ueq  = 0;
            info.ur   = u;
            info.dhat = zeros(plant.n_states, 1);
            info.e    = e;
            info.edot = edot;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('eint', 0, 't_last', -inf, 'u_hold', 0);
        end
    end
end
