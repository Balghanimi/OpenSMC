classdef HighGainObserver < core.Estimator
    %HIGHGAINOBSERVER Second-order high-gain observer.
    %
    %   Estimates unmeasured states from output y = x1 using:
    %
    %       x_hat1_dot = x_hat2 - (alpha2/eps) * (x_hat1 - y)
    %       x_hat2_dot = -(alpha1/eps^2) * (x_hat1 - y)
    %
    %   The observer converges as eps -> 0 with error O(eps).
    %   Peaking phenomenon may occur; use small initial conditions.
    %
    %   State estimates are stored in obj.state.xhat (2 x 1).
    %   Returns dhat = 0 since HGO provides state estimates, not
    %   direct disturbance estimates. Access obj.state.xhat for
    %   estimated position and velocity.
    %
    %   Parameters:
    %       alpha1  - (scalar) gain for 2nd correction (default 9)
    %       alpha2  - (scalar) gain for 1st correction (default 6)
    %       epsilon - (scalar) observer bandwidth, small = fast (default 0.01)
    %       dt      - (scalar) integration step (default 1e-4)
    %
    %   Reference: Liu, J. & Wang, X. (2012). "Advanced Sliding Mode
    %   Control for Mechanical Systems", Ch 8.1-8.2, Springer.

    properties
        name = 'HighGainObserver'
    end

    methods
        function obj = HighGainObserver(varargin)
            p.alpha1  = 9;       % gain coefficient (ε^-2 term)
            p.alpha2  = 6;       % gain coefficient (ε^-1 term)
            p.epsilon = 0.01;    % observer bandwidth
            p.dt      = 1e-4;    % integration step
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('xhat', [0; 0]);
        end

        function dhat = estimate(obj, ~, x, ~, y)
            xh  = obj.state.xhat;
            eps = obj.params.epsilon;
            a1  = obj.params.alpha1;
            a2  = obj.params.alpha2;

            % Observer error
            e_obs = xh(1) - y(1);

            % Observer dynamics
            xh1_dot = xh(2) - a2/eps * e_obs;
            xh2_dot = -a1/(eps^2) * e_obs;

            % Euler integration
            dt_ = obj.params.dt;
            obj.state.xhat = xh + dt_ * [xh1_dot; xh2_dot];

            % HGO provides state estimates, not disturbance
            dhat = zeros(size(x));
        end

        function update(~, ~, ~, ~, ~, ~)
            % Integration is done in estimate(); nothing extra needed.
        end

        function reset(obj)
            obj.state = struct('xhat', [0; 0]);
        end
    end
end
