classdef LevantDifferentiator < core.Estimator
    %LEVANTDIFFERENTIATOR Arbitrary-order robust exact differentiator.
    %
    %   Computes the first n derivatives of a noisy signal using
    %   higher-order sliding mode (HOSM) differentiation.
    %
    %   For order n=2 (estimates f, fdot, fddot):
    %       z0dot = -lambda2 * L^(1/3) * |z0-f|^(2/3) * sign(z0-f) + z1
    %       z1dot = -lambda1 * L^(1/2) * |z1-z0dot|^(1/2) * sign(z1-z0dot) + z2
    %       z2dot = -lambda0 * L * sign(z2 - z1dot)
    %
    %   Output: z0 -> f,  z1 -> fdot,  z2 -> fddot  (exact in finite time)
    %
    %   The differentiator converges in finite time and provides exact
    %   derivatives for signals with bounded (n+1)-th derivative.
    %
    %   Parameters:
    %       order    - differentiation order (default 2)
    %       L        - Lipschitz constant bound (default 100)
    %       lambdas  - gain vector (default: optimal from Levant 2003)
    %       dt       - integration step (default 1e-4)
    %
    %   Reference:
    %       Levant, A. (2003). "Higher-order sliding modes, differentiation
    %       and output-feedback control." IJC, 76(9-10), 924-941.

    properties
        name = 'LevantDifferentiator'
    end

    methods
        function obj = LevantDifferentiator(varargin)
            p.order   = 2;
            p.L       = 100;
            p.lambdas = [];     % auto-set in first call
            p.dt      = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end

            % Default optimal gains (Levant 2003, Table 1)
            if isempty(p.lambdas)
                switch p.order
                    case 1
                        p.lambdas = [1.5, 1.1];
                    case 2
                        p.lambdas = [2.0, 1.5, 1.1];
                    case 3
                        p.lambdas = [3.0, 2.0, 1.5, 1.1];
                    case 4
                        p.lambdas = [5.0, 3.0, 2.0, 1.5, 1.1];
                    otherwise
                        % Generic gains (conservative)
                        p.lambdas = linspace(p.order+1, 1.1, p.order+1);
                end
            end

            obj.params = p;
            obj.state  = struct('z', []);
        end

        function dhat = estimate(obj, ~, x, ~, y)
            n = obj.params.order;
            L = obj.params.L;
            lam = obj.params.lambdas;
            dt_ = obj.params.dt;

            % Initialize state vector z = [z0, z1, ..., zn]
            if isempty(obj.state.z)
                obj.state.z = zeros(n+1, 1);
                if ~isempty(y)
                    obj.state.z(1) = y(1);
                end
            end

            z = obj.state.z;
            f = y(1);  % input signal

            % Compute differentiator dynamics (recursive structure)
            zdot = zeros(n+1, 1);
            sigma = z(1) - f;

            for k = 0:n
                alpha_k = (n - k) / (n + 1);

                if k < n
                    % Correction + next state
                    zdot(k+1) = -lam(k+1) * L^(1/(n-k+1)) ...
                                * abs(sigma)^alpha_k * sign(sigma) ...
                                + z(k+2);
                else
                    % Last stage: pure sign
                    zdot(k+1) = -lam(k+1) * L * sign(sigma);
                end

                % Update sigma for next stage
                if k < n
                    sigma = z(k+2) - zdot(k+1);
                end
            end

            % Euler integration
            obj.state.z = z + dt_ * zdot;

            % Return disturbance estimate (use 2nd derivative as lumped disturbance)
            n_x = numel(x);
            dhat = zeros(n_x, 1);
            if n >= 2
                dhat(min(n_x, 1)) = obj.state.z(3);  % fddot estimate
            elseif n >= 1
                dhat(min(n_x, 1)) = obj.state.z(2);  % fdot estimate
            end
        end

        function update(~, ~, ~, ~, ~, ~)
            % Integration done in estimate()
        end

        function reset(obj)
            obj.state = struct('z', []);
        end

        function derivatives = get_derivatives(obj)
            %GET_DERIVATIVES Return all estimated derivatives [f, fdot, fddot, ...].
            derivatives = obj.state.z;
        end
    end
end
