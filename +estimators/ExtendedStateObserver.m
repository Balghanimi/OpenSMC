classdef ExtendedStateObserver < core.Estimator
    %EXTENDEDSTATEOBSERVER Third-order ESO for lumped uncertainty estimation.
    %
    %   Estimates states x1, x2 and the total uncertainty f(t) using:
    %
    %       x_hat1_dot = x_hat2 + (alpha1/eps) * (y - x_hat1)
    %       x_hat2_dot = b*u + sigma_hat + (alpha2/eps^2) * (y - x_hat1)
    %       sigma_dot  = (alpha3/eps^3) * (y - x_hat1)
    %
    %   where sigma_hat -> f(t) as eps -> 0. The estimated lumped
    %   uncertainty is returned as dhat, suitable for feedforward
    %   disturbance compensation in the control law.
    %
    %   Peaking suppression: set 'peaking' to 'ramp' or 'sigmoid'
    %   to gradually increase the observer gain (Eq. 8.28, 8.29).
    %
    %   Parameters:
    %       alpha1  - (scalar) 1st gain (default 6)
    %       alpha2  - (scalar) 2nd gain (default 11)
    %       alpha3  - (scalar) 3rd gain (default 6)
    %       epsilon - (scalar) observer bandwidth, small = fast (default 0.01)
    %       b       - (scalar) input gain (plant-specific, default 1)
    %       peaking - (string) 'none'|'ramp'|'sigmoid' (default 'none')
    %       dt      - (scalar) integration step (default 1e-4)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 8.3-8.4, Springer.

    properties
        name = 'ExtendedStateObserver'
    end

    methods
        function obj = ExtendedStateObserver(varargin)
            p.alpha1  = 6;
            p.alpha2  = 11;
            p.alpha3  = 6;
            p.epsilon = 0.01;
            p.b       = 1;        % input gain of plant
            p.peaking = 'none';   % peaking suppression mode
            p.lambda  = 50;       % sigmoid rate (for peaking='sigmoid')
            p.dt      = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('xhat', [0; 0; 0], 't', 0);
        end

        function dhat = estimate(obj, t, x, u, y)
            xh  = obj.state.xhat;
            eps = obj.get_epsilon(t);
            a1  = obj.params.alpha1;
            a2  = obj.params.alpha2;
            a3  = obj.params.alpha3;
            b   = obj.params.b;

            % Observer correction
            e_obs = y(1) - xh(1);

            % ESO dynamics (Eq. 8.19-8.21)
            xh1_dot = xh(2) + a1/eps * e_obs;
            xh2_dot = b*u(1) + xh(3) + a2/(eps^2) * e_obs;
            xh3_dot = a3/(eps^3) * e_obs;

            % Euler integration
            dt_ = obj.params.dt;
            obj.state.xhat = xh + dt_ * [xh1_dot; xh2_dot; xh3_dot];
            obj.state.t = t;

            % Return estimated lumped uncertainty as disturbance
            n = numel(x);
            dhat = zeros(n, 1);
            dhat(min(n, 1)) = obj.state.xhat(3);  % sigma_hat -> f(t)
        end

        function update(~, ~, ~, ~, ~, ~)
            % Integration is done in estimate().
        end

        function reset(obj)
            obj.state = struct('xhat', [0; 0; 0], 't', 0);
        end
    end

    methods (Access = private)
        function eps = get_epsilon(obj, t)
            %GET_EPSILON Compute effective epsilon with peaking suppression.
            switch obj.params.peaking
                case 'ramp'
                    % Eq. 8.28: R = 100*t^3 for t<=1, else 100
                    if t <= 1
                        R = 100 * t^3;
                    else
                        R = 100;
                    end
                    eps = max(1/R, obj.params.epsilon);
                case 'sigmoid'
                    % Eq. 8.29: R = mu*(1-exp(-lam1*t))/(1+exp(-lam2*t))
                    mu  = 100;
                    lam = obj.params.lambda;
                    R   = mu * (1 - exp(-lam*t)) / (1 + exp(-lam*t));
                    eps = max(1/max(R, 1e-6), obj.params.epsilon);
                otherwise
                    eps = obj.params.epsilon;
            end
        end
    end
end
