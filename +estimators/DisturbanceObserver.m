classdef DisturbanceObserver < core.Estimator
    %DISTURBANCEOBSERVER Slow time-varying disturbance observer.
    %
    %   Estimates a slowly varying disturbance d(t) for a second-order
    %   system theta_ddot = -b*theta_dot + a*u - d using:
    %
    %       d_hat_dot    = k1 * (omega_hat - theta_dot)
    %       omega_hat_dot = -d_hat + a*u - k2*(omega_hat - theta_dot) - b*theta_dot
    %
    %   where k1, k2 > 0 are observer gains. The Lyapunov stability
    %   analysis guarantees d_hat -> d as k1, k2 -> infinity.
    %
    %   Returns dhat = estimated disturbance, directly usable for
    %   feedforward compensation in the SMC control law.
    %
    %   Parameters:
    %       k1    - (scalar) disturbance estimation gain (default 500)
    %       k2    - (scalar) velocity estimation gain (default 200)
    %       a     - (scalar) input coefficient (default 5)
    %       b     - (scalar) damping coefficient (default 0.15)
    %       dt    - (scalar) integration step (default 1e-4)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 8.7-8.8, Springer.

    properties
        name = 'DisturbanceObserver'
    end

    methods
        function obj = DisturbanceObserver(varargin)
            p.k1 = 500;     % disturbance gain
            p.k2 = 200;     % velocity gain
            p.a  = 5;       % input coefficient
            p.b  = 0.15;    % damping coefficient
            p.dt = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('dhat', 0, 'omega_hat', 0);
        end

        function dhat = estimate(obj, ~, x, u, ~)
            dh = obj.state.dhat;
            wh = obj.state.omega_hat;
            k1 = obj.params.k1;
            k2 = obj.params.k2;
            a  = obj.params.a;
            b  = obj.params.b;

            % x(2) = theta_dot (velocity)
            theta_dot = x(min(2, numel(x)));

            % Observer dynamics (Eq. 8.52-8.53)
            dh_dot = k1 * (wh - theta_dot);
            wh_dot = -dh + a*u(1) - k2*(wh - theta_dot) - b*theta_dot;

            % Euler integration
            dt_ = obj.params.dt;
            obj.state.dhat     = dh + dt_ * dh_dot;
            obj.state.omega_hat = wh + dt_ * wh_dot;

            % Return disturbance estimate
            n = numel(x);
            dhat = zeros(n, 1);
            dhat(min(n, 1)) = obj.state.dhat;
        end

        function update(~, ~, ~, ~, ~, ~)
            % Integration is done in estimate().
        end

        function reset(obj)
            obj.state = struct('dhat', 0, 'omega_hat', 0);
        end
    end
end
