classdef Quadrotor6DOF < core.Plant
    %QUADROTOR6DOF 6-DOF quadrotor dynamics model.
    %
    %   Full nonlinear quadrotor with translational and rotational dynamics.
    %   Standard Newton-Euler formulation with Euler angle kinematics.
    %
    %   State: x = [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]' (12x1)
    %     (x,y,z)           - inertial position [m]
    %     (phi,theta,psi)   - roll, pitch, yaw [rad]
    %     (vx,vy,vz)        - inertial velocity [m/s]
    %     (p,q,r)           - body angular rates [rad/s]
    %
    %   Input: u = [U1, U2, U3, U4]' (4x1)
    %     U1 - total thrust [N]
    %     U2 - roll torque [N*m]
    %     U3 - pitch torque [N*m]
    %     U4 - yaw torque [N*m]
    %
    %   Parameters:
    %       m   - (scalar) mass [kg] (default 0.468)
    %       g   - (scalar) gravity [m/s^2] (default 9.81)
    %       Ixx - (scalar) roll inertia [kg*m^2] (default 4.856e-3)
    %       Iyy - (scalar) pitch inertia [kg*m^2] (default 4.856e-3)
    %       Izz - (scalar) yaw inertia [kg*m^2] (default 8.801e-3)
    %       l   - (scalar) arm length [m] (default 0.225)
    %
    %   Reference: Quadrotor model from RBF-ELM-ITSMC manuscript.

    properties
        name             = 'Quadrotor6DOF'
        n_states         = 12
        n_inputs         = 4
        n_outputs        = 6
        n_dof            = 6
        is_underactuated = true
    end

    methods
        function obj = Quadrotor6DOF(varargin)
            p.m   = 0.468;
            p.g   = 9.81;
            p.Ixx = 4.856e-3;
            p.Iyy = 4.856e-3;
            p.Izz = 8.801e-3;
            p.l   = 0.225;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = zeros(12, 1);
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;

            phi = x(4); theta = x(5); psi = x(6);
            vx  = x(7); vy    = x(8); vz  = x(9);
            pp  = x(10); qq   = x(11); rr  = x(12);

            U1 = u(1); U2 = u(2); U3 = u(3); U4 = u(4);

            % Translational dynamics
            ax = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * U1/p.m;
            ay = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * U1/p.m;
            az = -p.g + cos(phi)*cos(theta) * U1/p.m;

            % Rotational dynamics
            dp = ((p.Iyy - p.Izz) * qq * rr + U2) / p.Ixx;
            dq = ((p.Izz - p.Ixx) * pp * rr + U3) / p.Iyy;
            dr = ((p.Ixx - p.Iyy) * pp * qq + U4) / p.Izz;

            % Euler angle kinematics
            phi_dot   = pp + qq*sin(phi)*tan(theta) + rr*cos(phi)*tan(theta);
            theta_dot = qq*cos(phi) - rr*sin(phi);
            if abs(cos(theta)) > 1e-6
                psi_dot = qq*sin(phi)/cos(theta) + rr*cos(phi)/cos(theta);
            else
                psi_dot = 0;
            end

            xdot = [vx; vy; vz; phi_dot; theta_dot; psi_dot; ...
                    ax; ay; az; dp; dq; dr];

            % Add disturbance (first 12 or fewer elements)
            nd = min(numel(d), 12);
            xdot(1:nd) = xdot(1:nd) + d(1:nd);
        end

        function y = output(~, x)
            y = x(1:6);  % position + attitude
        end

        function [e, edot] = get_error(~, x, xref)
            % Position + attitude error
            e    = xref(1:6) - x(1:6);
            edot = xref(7:12) - x(7:12);
        end
    end
end
