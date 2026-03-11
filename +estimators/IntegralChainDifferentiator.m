classdef IntegralChainDifferentiator < core.Estimator
    %INTEGRALCHAINDIFFERENTIATOR High-order integral-chain differentiator.
    %
    %   Estimates all unmeasured states and the lumped uncertainty f(x)
    %   from the output y = x1 using the integral-chain structure:
    %
    %       x_hat1_dot = x_hat2
    %       x_hat2_dot = x_hat3
    %       ...
    %       x_hatn_dot = x_hat{n+1}
    %       x_hat{n+1}_dot = -(a1/eps^{n+1})(x_hat1 - y) - (a2/eps^n)x_hat2
    %                        - ... - (a_{n+1}/eps)x_hat{n+1}
    %
    %   As eps -> 0: x_hat_i -> x_i (i=1..n) and x_hat{n+1} -> f(x)+g(x)*u.
    %   The polynomial s^{n+1} + a_{n+1}*s^n + ... + a1 = 0 must be Hurwitz.
    %
    %   Returns dhat based on the (n+1)-th state estimate (total uncertainty).
    %
    %   Parameters:
    %       n       - (int) system order (default 2)
    %       a       - (1 x n+1) Hurwitz coefficients (default [1 3 3])
    %       epsilon - (scalar) bandwidth parameter (default 0.01)
    %       dt      - (scalar) integration step (default 1e-4)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 8.5-8.6, Springer.

    properties
        name = 'IntegralChainDifferentiator'
    end

    methods
        function obj = IntegralChainDifferentiator(varargin)
            p.n       = 2;           % system order
            p.a       = [1, 3, 3];   % Hurwitz coefficients (n+1 values)
            p.epsilon = 0.01;
            p.dt      = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            % Ensure coefficient vector length matches n+1
            if numel(p.a) ~= p.n + 1
                error('OpenSMC:ICD', 'Length of a must be n+1 = %d', p.n+1);
            end
            obj.params = p;
            obj.state  = struct('xhat', zeros(p.n + 1, 1));
        end

        function dhat = estimate(obj, ~, x, ~, y)
            xh  = obj.state.xhat;
            n   = obj.params.n;
            eps = obj.params.epsilon;
            a   = obj.params.a;

            % Build dynamics (Eq. 8.37)
            xh_dot = zeros(n+1, 1);

            % Chain: x_hat_i_dot = x_hat_{i+1} for i = 1..n
            for i = 1:n
                xh_dot(i) = xh(i+1);
            end

            % Last state: correction term
            e_obs = xh(1) - y(1);
            correction = 0;
            for j = 1:n+1
                correction = correction + a(j) / (eps^(n+2-j)) * xh(max(j,1));
            end
            % Rewrite: -(a1/eps^{n+1})*e_obs - (a2/eps^n)*xhat2 - ...
            xh_dot(n+1) = -a(1)/(eps^(n+1)) * e_obs;
            for j = 2:n+1
                xh_dot(n+1) = xh_dot(n+1) - a(j)/(eps^(n+2-j)) * xh(j);
            end

            % Euler integration
            dt_ = obj.params.dt;
            obj.state.xhat = xh + dt_ * xh_dot;

            % Return estimated uncertainty (last state)
            nx = numel(x);
            dhat = zeros(nx, 1);
            dhat(min(nx, 1)) = obj.state.xhat(n+1);
        end

        function update(~, ~, ~, ~, ~, ~)
            % Integration is done in estimate().
        end

        function reset(obj)
            obj.state = struct('xhat', zeros(obj.params.n + 1, 1));
        end
    end
end
