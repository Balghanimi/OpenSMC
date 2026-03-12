classdef CascadedSMC < core.Architecture
    %CASCADEDSMC Cascaded architecture for underactuated aerial vehicles.
    %
    %   Two-loop cascade structure:
    %     Outer loop (position): ITSMC + RBF-ELM → desired accelerations
    %     Control allocation: accelerations → thrust + desired angles
    %     Inner loop (attitude): PD controller → torques
    %
    %   This architecture is standard for quadrotor control where position
    %   control produces virtual commands that are mapped to physical
    %   actuator inputs through control allocation.
    %
    %   Controllers cell array:
    %     controllers{1} = outer_x (ITSMC for x-axis)
    %     controllers{2} = outer_y (ITSMC for y-axis)
    %     controllers{3} = outer_z (ITSMC for z-axis)
    %
    %   Parameters:
    %       Kp_att  - (scalar) attitude proportional gain (default 15)
    %       Kd_att  - (scalar) attitude derivative gain (default 5)
    %       u1_min  - (scalar) minimum thrust [N] (default 0.1*m*g)
    %       u1_max  - (scalar) maximum thrust [N] (default 2.5*m*g)
    %       angle_max - (scalar) max tilt angle [rad] (default pi/6)
    %
    %   Reference: Al Ghanimi (2026). "RBF-ELM-ITSMC Hybrid Controller
    %   for Quadrotor UAV Trajectory Tracking." (Manuscript)

    properties
        name = 'Cascaded'
    end

    methods
        function obj = CascadedSMC(ctrl_x, ctrl_y, ctrl_z)
            obj.controllers = {ctrl_x, ctrl_y, ctrl_z};
            p.Kp_att    = 15;
            p.Kd_att    = 5;
            p.angle_max = pi/6;
            obj.params  = p;
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p_arch = obj.params;
            pp     = plant.params;

            % --- Outer loop: ITSMC per axis ---
            % X-axis controller
            x_state = [x(1); x(7)];
            x_ref   = [xref(1); xref(7)];
            plant_x = make_siso_plant(plant, 1, 7);
            [ax_des, info_x] = obj.controllers{1}.compute(t, x_state, x_ref, plant_x);
            ax_des = ax_des(1);

            % Y-axis controller
            y_state = [x(2); x(8)];
            y_ref   = [xref(2); xref(8)];
            plant_y = make_siso_plant(plant, 2, 8);
            [ay_des, info_y] = obj.controllers{2}.compute(t, y_state, y_ref, plant_y);
            ay_des = ay_des(1);

            % Z-axis controller
            z_state = [x(3); x(9)];
            z_ref   = [xref(3); xref(9)];
            plant_z = make_siso_plant(plant, 3, 9);
            [az_des, info_z] = obj.controllers{3}.compute(t, z_state, z_ref, plant_z);
            az_des = az_des(1);

            % --- Control allocation ---
            psi_ref = 0;
            if numel(xref) >= 6
                psi_ref = xref(6);
            end

            % Thrust magnitude
            % Guard against negative Z-force demand (az_des + g < 0)
            % which would cause sqrt to produce thrust in the wrong
            % direction.  When this occurs, reduce to minimum thrust.
            if az_des + pp.g < 0
                U1 = 0.1 * pp.m * pp.g;
            else
                U1 = pp.m * sqrt(ax_des^2 + ay_des^2 + (az_des + pp.g)^2);
                U1 = max(0.1 * pp.m * pp.g, min(U1, 2.5 * pp.m * pp.g));
            end

            % Desired roll (phi)
            arg_phi = pp.m * (ax_des*sin(psi_ref) - ay_des*cos(psi_ref)) / U1;
            arg_phi = max(-0.95, min(0.95, arg_phi));
            phi_d = asin(arg_phi);
            phi_d = max(-p_arch.angle_max, min(p_arch.angle_max, phi_d));

            % Desired pitch (theta)
            cos_phi_d = max(abs(cos(phi_d)), 0.01) * sign(cos(phi_d) + 1e-10);
            arg_theta = pp.m * (ax_des*cos(psi_ref) + ay_des*sin(psi_ref)) / (U1 * cos_phi_d);
            arg_theta = max(-0.95, min(0.95, arg_theta));
            theta_d = asin(arg_theta);
            theta_d = max(-p_arch.angle_max, min(p_arch.angle_max, theta_d));

            % --- Inner loop: PD attitude control ---
            U2 = pp.Ixx * (p_arch.Kp_att * (phi_d   - x(4)) + p_arch.Kd_att * (-x(10)));
            U3 = pp.Iyy * (p_arch.Kp_att * (theta_d - x(5)) + p_arch.Kd_att * (-x(11)));
            U4 = pp.Izz * (p_arch.Kp_att * (psi_ref - x(6)) + p_arch.Kd_att * (-x(12)));

            u = [U1; U2; U3; U4];

            % Diagnostics
            info.ax_des  = ax_des;
            info.ay_des  = ay_des;
            info.az_des  = az_des;
            info.phi_d   = phi_d;
            info.theta_d = theta_d;
            info.U1      = U1;
            info.sx      = info_x.s;
            info.sy      = info_y.s;
            info.sz      = info_z.s;
            info.dhat_x  = info_x.dhat;
            info.dhat_y  = info_y.dhat;
            info.dhat_z  = info_z.dhat;
            info.s       = [info_x.s; info_y.s; info_z.s];
        end

        function reset(obj)
            reset@core.Architecture(obj);
        end
    end
end

function sp = make_siso_plant(plant, pos_idx, vel_idx)
    %MAKE_SISO_PLANT Create a lightweight SISO wrapper for per-axis control.
    sp = struct();
    sp.n_inputs = 1;
    sp.n_states = 2;
    sp.get_error = @(x, xref) deal(xref(1) - x(1), xref(2) - x(2));
    sp.output = @(x) x(1);
end
