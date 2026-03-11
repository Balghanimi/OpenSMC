classdef PIDSurface < core.SlidingSurface
    %PIDSURFACE PID-type sliding surface for second-order SMC.
    %
    %   Incorporates integral action into the sliding surface:
    %
    %       s = alpha * edot + beta * e + gamma * integral(e)
    %
    %   This surface is used with second-order SMC where the controller
    %   computes du/dt (control derivative) instead of u directly,
    %   producing a continuous control signal.
    %
    %   The integral term improves steady-state accuracy and disturbance
    %   rejection. The three coefficients alpha, beta, gamma provide
    %   PID-like tuning of the sliding dynamics.
    %
    %   Parameters:
    %       alpha - (1 x n) derivative coefficients (default [-10 0.5])
    %       beta  - (1 x n) proportional coefficients (default [-10 35])
    %       gamma - (1 x n) integral coefficients (default [0.01 0.01])
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Ch 3, Appendix E, Springer.
    %   Also: Chiang et al. (2011), "Second-order sliding mode control
    %   for a magnetic levitation system."

    properties
        name = 'PID'
    end

    methods
        function obj = PIDSurface(varargin)
            p.alpha = [-10, 0.5];
            p.beta  = [-10, 35];
            p.gamma = [0.01, 0.01];
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, eint, ~)
            p = obj.params;
            n = numel(e);

            % Ensure coefficient vectors match error dimension
            alpha = p.alpha(:);
            beta  = p.beta(:);
            gamma = p.gamma(:);

            if numel(alpha) < n, alpha = [alpha; zeros(n-numel(alpha), 1)]; end
            if numel(beta)  < n, beta  = [beta;  zeros(n-numel(beta),  1)]; end
            if numel(gamma) < n, gamma = [gamma; zeros(n-numel(gamma), 1)]; end

            alpha = alpha(1:n);
            beta  = beta(1:n);
            gamma = gamma(1:n);

            % PID-type surface: s = alpha' * edot + beta' * e + gamma' * eint
            s = alpha' * edot + beta' * e + gamma' * eint;
        end
    end
end
