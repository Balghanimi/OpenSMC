classdef SurfaceVessel < core.Plant
    %SURFACEVESSEL 3-DOF marine surface vessel (surge, sway, yaw).
    %
    %   Standard Fossen model in body-fixed frame:
    %       M * nu_dot + C(nu) * nu + D * nu = tau + d
    %       eta_dot = R(psi) * nu
    %
    %   State: x = [x_n; y_n; psi; u_b; v_b; r]
    %     x_n, y_n = north-east position [m]
    %     psi      = heading [rad]
    %     u_b      = surge velocity [m/s]
    %     v_b      = sway velocity [m/s]
    %     r        = yaw rate [rad/s]
    %
    %   Input: u = [tau_u; tau_v; tau_r]  (surge force, sway force, yaw moment)
    %   Output: y = [x_n; y_n; psi]
    %
    %   Underactuated variant: set n_inputs = 2, only surge + yaw
    %   (no sway thruster), which is the common ship configuration.
    %
    %   Reference:
    %       Fossen, T.I. (2011). "Handbook of Marine Craft Hydrodynamics
    %       and Motion Control", Wiley, Chapter 6.

    properties
        name             = 'SurfaceVessel'
        n_states         = 6
        n_inputs         = 3
        n_outputs        = 3
        n_dof            = 3
        is_underactuated = false
    end

    methods
        function obj = SurfaceVessel(varargin)
            p.m    = 23.8;    % vessel mass [kg] (scaled model)
            p.Iz   = 1.76;   % yaw moment of inertia [kg*m^2]
            p.xg   = 0.046;  % CG offset from CO [m]
            p.Xu   = -0.72;  % surge linear damping [N*s/m]
            p.Yv   = -0.89;  % sway linear damping [N*s/m]
            p.Nr   = -1.90;  % yaw linear damping [N*m*s/rad]
            p.Xuu  = -1.33;  % surge quadratic damping
            p.Yvv  = -36.5;  % sway quadratic damping
            p.Nrr  = -0.75;  % yaw quadratic damping
            p.Xudot = -2.0;  % surge added mass [kg]
            p.Yvdot = -10.0; % sway added mass [kg]
            p.Nrdot = -1.0;  % yaw added inertia [kg*m^2]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = zeros(6, 1);
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            psi = x(3);
            ub  = x(4);
            vb  = x(5);
            r   = x(6);

            % Rotation matrix (body -> NED)
            cp = cos(psi);
            sp = sin(psi);
            R  = [cp, -sp, 0;
                  sp,  cp, 0;
                  0,    0, 1];

            % Kinematics: eta_dot = R(psi) * nu
            eta_dot = R * [ub; vb; r];

            % Mass matrix (rigid-body + added mass)
            m11 = p.m - p.Xudot;
            m22 = p.m - p.Yvdot;
            m33 = p.Iz - p.Nrdot;
            m23 = p.m * p.xg;  % coupling term

            % Coriolis + centripetal forces (simplified for 3-DOF)
            c13 = -(p.m - p.Yvdot) * vb - p.m * p.xg * r;
            c23 =  (p.m - p.Xudot) * ub;

            % Damping (linear + quadratic)
            d1 = -p.Xu * ub - p.Xuu * abs(ub) * ub;
            d2 = -p.Yv * vb - p.Yvv * abs(vb) * vb;
            d3 = -p.Nr * r  - p.Nrr * abs(r)  * r;

            % Forces in body frame
            tau = u(1:3);

            % Solve M * nu_dot = tau - C*nu - D*nu + d_body
            d_body = d(4:6);  % disturbance on velocities
            rhs1 = tau(1) + c13*r       - d1 + d_body(1);
            rhs2 = tau(2) + c23*r       - d2 + d_body(2);
            rhs3 = tau(3) - c13*0 - c23*0 - d3 + d_body(3);

            % Simplified decoupled solve (neglecting small m23 coupling)
            nu_dot = [rhs1 / m11;
                      rhs2 / m22;
                      rhs3 / m33];

            xdot = [eta_dot; nu_dot] + [d(1:3); 0; 0; 0];
        end

        function y = output(~, x)
            y = x(1:3);  % [x_n; y_n; psi]
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref(1:3) - x(1:3);
            edot = xref(4:6) - x(4:6);
        end
    end
end
