classdef InvertedPendulum < core.Plant
    %INVERTEDPENDULUM Cart-pole system (underactuated benchmark).
    %
    %   Classic underactuated system: force applied to cart,
    %   pendulum must be stabilized upright.
    %
    %   State: x = [cart_pos; theta; cart_vel; theta_dot]
    %   Input: u = horizontal force on cart
    %   Underactuated: 2 DOF, 1 input

    properties
        name             = 'InvertedPendulum'
        n_states         = 4
        n_inputs         = 1
        n_outputs        = 2
        n_dof            = 2
        is_underactuated = true
    end

    methods
        function obj = InvertedPendulum(varargin)
            p.M  = 1.0;    % cart mass [kg]
            p.m  = 0.1;    % pendulum mass [kg]
            p.l  = 0.5;    % pendulum half-length [m]
            p.g  = 9.81;   % gravity [m/s^2]
            p.b  = 0.1;    % cart friction [N*s/m]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = [0; pi/6; 0; 0];  % pendulum tilted 30 deg
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            theta    = x(2);
            xdot_val = x(3);
            thetadot = x(4);

            st = sin(theta);
            ct = cos(theta);
            D  = p.M + p.m - p.m * ct^2;

            xddot     = (u - p.b*xdot_val + p.m*p.l*thetadot^2*st ...
                         - p.m*p.g*st*ct + d(1)) / D;
            thetaddot = (-u*ct + p.b*xdot_val*ct ...
                         - p.m*p.l*thetadot^2*st*ct ...
                         + (p.M + p.m)*p.g*st + d(2)) / (p.l * D);

            xdot = [xdot_val; thetadot; xddot; thetaddot];
        end

        function y = output(~, x)
            y = x(1:2);  % cart position and angle
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref(1:2) - x(1:2);
            edot = xref(3:4) - x(3:4);
        end
    end
end
