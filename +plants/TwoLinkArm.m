classdef TwoLinkArm < core.Plant
    %TWOLINKARM 2-DOF planar robot manipulator (fully actuated benchmark).
    %
    %   Standard Euler-Lagrange formulation:
    %       M(q) * qddot + C(q, qdot) * qdot + G(q) = tau + d
    %
    %   State: x = [q1; q1dot; q2; q2dot]
    %   Input: u = [tau1; tau2]  (joint torques)
    %   Output: y = [q1; q2]    (joint angles)
    %
    %   The most widely used benchmark plant for SMC on robot manipulators.
    %   Fully actuated (2 inputs, 2 DOF) so DirectSMC architecture applies.
    %
    %   Parameters follow the standard formulation from:
    %       Slotine, J.J. & Li, W. (1991). "Applied Nonlinear Control",
    %       Prentice-Hall, Chapter 9.
    %
    %   Default values represent a typical small laboratory arm.

    properties
        name             = 'TwoLinkArm'
        n_states         = 4
        n_inputs         = 2
        n_outputs        = 2
        n_dof            = 2
        is_underactuated = false
    end

    methods
        function obj = TwoLinkArm(varargin)
            p.m1  = 1.0;     % link 1 mass [kg]
            p.m2  = 1.0;     % link 2 mass [kg]
            p.l1  = 1.0;     % link 1 length [m]
            p.l2  = 1.0;     % link 2 length [m]
            p.lc1 = 0.5;     % link 1 center of mass [m]
            p.lc2 = 0.5;     % link 2 center of mass [m]
            p.I1  = 0.083;   % link 1 inertia [kg*m^2]  (1/12 * m * l^2)
            p.I2  = 0.083;   % link 2 inertia [kg*m^2]
            p.g   = 9.81;    % gravity [m/s^2]
            p.b1  = 0.0;     % joint 1 friction [N*m*s]
            p.b2  = 0.0;     % joint 2 friction [N*m*s]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = [0; 0; 0; 0];
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            q1    = x(1);
            q1dot = x(2);
            q2    = x(3);
            q2dot = x(4);

            % Trigonometric terms
            c2  = cos(q2);
            s2  = sin(q2);
            c1  = cos(q1);
            c12 = cos(q1 + q2);

            % Inertia matrix M(q)
            alpha = p.m2 * p.l1 * p.lc2;
            M11 = p.m1*p.lc1^2 + p.I1 + p.m2*(p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2*c2) + p.I2;
            M12 = p.m2*(p.lc2^2 + p.l1*p.lc2*c2) + p.I2;
            M22 = p.m2*p.lc2^2 + p.I2;

            % Coriolis/centrifugal vector C(q,qdot)*qdot
            % Using Christoffel symbols (Spong et al. 2006, Eq. 6.44-6.45)
            h = alpha * s2;
            c1_val = -2*h*q1dot*q2dot - h*q2dot^2;   % C(q,qdot)*qdot row 1
            c2_val =  h*q1dot^2;                       % C(q,qdot)*qdot row 2

            % Gravity vector G(q)
            g1 = (p.m1*p.lc1 + p.m2*p.l1)*p.g*c1 + p.m2*p.lc2*p.g*c12;
            g2 = p.m2*p.lc2*p.g*c12;

            % Friction
            f1 = p.b1 * q1dot;
            f2 = p.b2 * q2dot;

            % M(q) * qddot = tau - C(q,qdot)*qdot - G(q) - friction + d
            tau = u(1:2);
            rhs = tau - [c1_val; c2_val] - [g1; g2] - [f1; f2] + d([2, 4]);

            % Solve M * qddot = rhs
            det_M = M11*M22 - M12^2;
            q1ddot = ( M22*rhs(1) - M12*rhs(2)) / det_M;
            q2ddot = (-M12*rhs(1) + M11*rhs(2)) / det_M;

            xdot = [q1dot; q1ddot; q2dot; q2ddot];
        end

        function y = output(~, x)
            y = [x(1); x(3)];  % joint angles
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref([1,3]) - x([1,3]);
            edot = xref([2,4]) - x([2,4]);
        end

        function [M, C, G] = get_dynamics_matrices(obj, x)
            %GET_DYNAMICS_MATRICES Return M(q), C(q,qdot)*qdot, G(q).
            %   Useful for computing equivalent control in SMC.
            p = obj.params;
            q1    = x(1);
            q1dot = x(2);
            q2    = x(3);
            q2dot = x(4);

            c2  = cos(q2);
            s2  = sin(q2);
            c1  = cos(q1);
            c12 = cos(q1 + q2);

            alpha = p.m2 * p.l1 * p.lc2;
            M11 = p.m1*p.lc1^2 + p.I1 + p.m2*(p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2*c2) + p.I2;
            M12 = p.m2*(p.lc2^2 + p.l1*p.lc2*c2) + p.I2;
            M22 = p.m2*p.lc2^2 + p.I2;
            M   = [M11, M12; M12, M22];

            h = alpha * s2;
            C = [-2*h*q1dot*q2dot - h*q2dot^2;
                  h*q1dot^2];

            g1 = (p.m1*p.lc1 + p.m2*p.l1)*p.g*c1 + p.m2*p.lc2*p.g*c12;
            g2 = p.m2*p.lc2*p.g*c12;
            G  = [g1; g2];
        end
    end
end
