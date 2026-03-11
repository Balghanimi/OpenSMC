classdef DoublePendulumCrane < core.Plant
    %DOUBLEPENDULUMCRANE Double-pendulum overhead crane system.
    %
    %   Underactuated system with trolley, hook, and payload.
    %   Control input is the horizontal force on the trolley.
    %
    %   State: x = [x_trolley; x_dot; phi; phi_dot; theta; theta_dot]
    %     phi   = hook angle (first pendulum)
    %     theta = payload angle (second pendulum)
    %   Input: u = horizontal force on trolley [N]
    %
    %   Dynamics (Qian & Yi 2015, Eq. 2.46):
    %       M(q)*q_ddot + C(q,q_dot)*q_dot + G(q) = [u; 0; 0]
    %   where q = [x; phi; theta]
    %
    %   Parameters:
    %       mt - (scalar) trolley mass [kg] (default 50)
    %       mc - (scalar) payload mass [kg] (default 2)
    %       mh - (scalar) hook mass [kg] (default 10)
    %       l1 - (scalar) upper cable length [m] (default 3)
    %       l2 - (scalar) lower cable length [m] (default 0.3)
    %       b  - (scalar) trolley friction [N*s/m] (default 0)
    %       g  - (scalar) gravity [m/s^2] (default 9.81)
    %
    %   Reference: Qian, D. & Yi, J. (2015). "Hierarchical Sliding Mode
    %   Control for Under-actuated Cranes", Ch 2-3, Springer.

    properties
        name             = 'DoublePendulumCrane'
        n_states         = 6
        n_inputs         = 1
        n_outputs        = 3
        n_dof            = 3
        is_underactuated = true
    end

    methods
        function obj = DoublePendulumCrane(varargin)
            p.mt = 50;     % trolley mass [kg]
            p.mc = 2;      % payload mass [kg]
            p.mh = 10;     % hook mass [kg]
            p.l1 = 3;      % upper cable length [m]
            p.l2 = 0.3;    % lower cable length [m]
            p.b  = 0;      % trolley friction [N*s/m]
            p.g  = 9.81;   % gravity [m/s^2]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = [0; 0; 0; 0; 0; 0];
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            dx     = x(2);
            phi    = x(3);
            dphi   = x(4);
            theta  = x(5);
            dtheta = x(6);

            % Generalized coordinates
            q    = [x(1); phi; theta];
            dq   = [dx; dphi; dtheta];

            % Mass matrix M (3x3) (Qian Eq. 2.46 / Appendix B)
            M = [p.mt+p.mh+p.mc,   -(p.mh+p.mc)*p.l1*cos(phi),   -p.mc*p.l2*cos(theta);
                 -(p.mh+p.mc)*p.l1*cos(phi),  (p.mh+p.mc)*p.l1^2,  p.mc*p.l1*p.l2*cos(phi-theta);
                 -p.mc*p.l2*cos(theta),  p.mc*p.l1*p.l2*cos(phi-theta),  p.mc*p.l2^2];

            % Coriolis/centripetal matrix C (3x3)
            C = [p.b,  (p.mh+p.mc)*p.l1*sin(phi)*dphi,  p.mc*p.l2*sin(theta)*dtheta;
                 0,  0,  -p.mc*p.l1*p.l2*sin(phi-theta)*dtheta;
                 0,  p.mc*p.l1*p.l2*sin(phi-theta)*dphi,  0];

            % Gravity vector G (3x1)
            G = [0;
                 (p.mh+p.mc)*p.g*p.l1*sin(phi);
                 p.mc*p.g*p.l2*sin(theta)];

            % Input vector
            U = [u(1); 0; 0];

            % Solve M*q_ddot = U - G - C*dq
            ddq = M \ (U - G - C*dq);

            xdot = [dx; ddq(1); dphi; ddq(2); dtheta; ddq(3)] + d;
        end

        function y = output(~, x)
            y = [x(1); x(3); x(5)];  % trolley position, hook angle, payload angle
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref([1, 3, 5]) - x([1, 3, 5]);
            edot = xref([2, 4, 6]) - x([2, 4, 6]);
        end

        function [f, b_vec] = get_dynamics_components(obj, x)
            %GET_DYNAMICS_COMPONENTS Return f_i, b_i for each subsystem.
            %   q_ddot_i = f_i(x) + b_i(x)*u  (i = 1,2,3)
            %
            %   Outputs:
            %       f     - (3x1) drift terms
            %       b_vec - (3x1) input coupling terms
            p = obj.params;
            dx     = x(2);
            phi    = x(3);
            dphi   = x(4);
            theta  = x(5);
            dtheta = x(6);

            dq = [dx; dphi; dtheta];

            M = [p.mt+p.mh+p.mc,   -(p.mh+p.mc)*p.l1*cos(phi),   -p.mc*p.l2*cos(theta);
                 -(p.mh+p.mc)*p.l1*cos(phi),  (p.mh+p.mc)*p.l1^2,  p.mc*p.l1*p.l2*cos(phi-theta);
                 -p.mc*p.l2*cos(theta),  p.mc*p.l1*p.l2*cos(phi-theta),  p.mc*p.l2^2];

            C = [p.b,  (p.mh+p.mc)*p.l1*sin(phi)*dphi,  p.mc*p.l2*sin(theta)*dtheta;
                 0,  0,  -p.mc*p.l1*p.l2*sin(phi-theta)*dtheta;
                 0,  p.mc*p.l1*p.l2*sin(phi-theta)*dphi,  0];

            G = [0;
                 (p.mh+p.mc)*p.g*p.l1*sin(phi);
                 p.mc*p.g*p.l2*sin(theta)];

            Minv = inv(M);
            f     = Minv * (-G - C*dq);     % drift: q_ddot when u=0
            b_vec = Minv * [1; 0; 0];       % input coupling
        end
    end
end
