classdef SinglePendulumCrane < core.Plant
    %SINGLEPENDULUMCRANE Single-pendulum overhead crane system.
    %
    %   Underactuated system with trolley (actuated) and payload (unactuated).
    %   Control input is the horizontal force on the trolley.
    %
    %   State: x = [x_trolley; x_dot; theta; theta_dot]
    %   Input: u = horizontal force on trolley [N]
    %
    %   Dynamics (Qian & Yi 2015, Eq. 2.16):
    %       x_ddot     = [u + m*l*theta_dot^2*sin(theta)]*l
    %                     + g*m*l*cos(theta)*sin(theta)] / [(M+m)*l - m*l*cos^2(theta)]
    %       theta_ddot = [u + m*l*theta_dot^2*sin(theta)]*cos(theta)
    %                     + g*(M+m)*sin(theta)] / [m*l*cos^2(theta) - (M+m)*l]
    %
    %   Parameters:
    %       M - (scalar) trolley mass [kg] (default 1.0)
    %       m - (scalar) payload mass [kg] (default 0.8)
    %       l - (scalar) cable length [m] (default 0.305)
    %       g - (scalar) gravity [m/s^2] (default 9.81)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Ch 2-3, Springer.

    properties
        name             = 'SinglePendulumCrane'
        n_states         = 4
        n_inputs         = 1
        n_outputs        = 2
        n_dof            = 2
        is_underactuated = true
    end

    methods
        function obj = SinglePendulumCrane(varargin)
            p.M = 1.0;     % trolley mass [kg]
            p.m = 0.8;     % payload mass [kg]
            p.l = 0.305;   % cable length [m]
            p.g = 9.81;    % gravity [m/s^2]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = [0; 0; 0; 0];
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            theta    = x(3);
            dx       = x(2);
            dtheta   = x(4);

            st = sin(theta);
            ct = cos(theta);

            A_ = p.M + p.m;
            B_ = p.m * p.l * ct;

            % Denominator terms
            den1 = A_ * p.l - B_ * ct;   % = (M+m)*l - m*l*cos^2(theta)
            den2 = B_ * ct - A_ * p.l;   % = m*l*cos^2(theta) - (M+m)*l

            % Trolley acceleration (from Qian Appendix A SPCrane.m)
            xddot = ((u(1) + p.m*p.l*dtheta^2*st)*p.l ...
                      + p.g*B_*st) / den1;

            % Pendulum acceleration
            thetaddot = ((u(1) + p.m*p.l*dtheta^2*st)*ct ...
                          + p.g*A_*st) / den2;

            xdot = [dx; xddot; dtheta; thetaddot] + d;
        end

        function y = output(~, x)
            y = [x(1); x(3)];  % trolley position and swing angle
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref([1, 3]) - x([1, 3]);
            edot = xref([2, 4]) - x([2, 4]);
        end

        function [f1, f2, b1, b2] = get_dynamics_components(obj, x)
            %GET_DYNAMICS_COMPONENTS Return f1, f2, b1, b2 for control law.
            %   x_ddot     = f1(x) + b1(x)*u
            %   theta_ddot = f2(x) + b2(x)*u
            p = obj.params;
            theta  = x(3);
            dtheta = x(4);

            st = sin(theta);
            ct = cos(theta);

            A_ = p.M + p.m;
            B_ = p.m * p.l * ct;
            D_ = p.m * p.l * st;

            den1 = A_ * p.l - B_ * ct;
            den2 = B_ * ct - A_ * p.l;

            % f1, b1: trolley subsystem
            f1 = (D_ * dtheta^2 * p.l + p.g * B_ * st) / den1;
            b1 = p.l / den1;

            % f2, b2: pendulum subsystem
            f2 = (D_ * dtheta^2 * ct + p.g * A_ * st) / den2;
            b2 = ct / den2;
        end
    end
end
