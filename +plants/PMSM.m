classdef PMSM < core.Plant
    %PMSM Permanent Magnet Synchronous Motor in dq-frame.
    %
    %   Standard dq-axis model (Park transform, surface-mounted PMSM):
    %
    %       L_d * di_d/dt = v_d - R_s * i_d + p * omega * L_q * i_q
    %       L_q * di_q/dt = v_q - R_s * i_q - p * omega * (L_d * i_d + psi_f)
    %       J * domega/dt = T_e - B * omega - T_L
    %       dtheta/dt     = omega
    %
    %   where T_e = 1.5 * p * (psi_f * i_q + (L_d - L_q) * i_d * i_q)
    %   For surface-mounted PMSM: L_d = L_q = L_s, so T_e = 1.5*p*psi_f*i_q
    %
    %   State: x = [i_d; i_q; omega; theta]
    %   Input: u = [v_d; v_q]  (dq-frame voltages)
    %   Output: y = [omega; theta]  (speed and position)
    %
    %   The most common SMC application in power electronics and drives.
    %   Speed/position control with current-loop SMC.
    %
    %   Parameters follow:
    %       Krause, P.C. et al. (2013). "Analysis of Electric Machinery
    %       and Drive Systems", 3rd ed., IEEE Press/Wiley.

    properties
        name             = 'PMSM'
        n_states         = 4
        n_inputs         = 2
        n_outputs        = 2
        n_dof            = 1
        is_underactuated = false
    end

    methods
        function obj = PMSM(varargin)
            p.Rs     = 1.2;      % stator resistance [Ohm]
            p.Ld     = 6.8e-3;   % d-axis inductance [H]
            p.Lq     = 6.8e-3;   % q-axis inductance [H] (= Ld for SPMSM)
            p.psi_f  = 0.175;    % permanent magnet flux linkage [Wb]
            p.pp     = 4;        % number of pole pairs
            p.J      = 0.003;    % rotor inertia [kg*m^2]
            p.B      = 0.001;    % viscous friction [N*m*s/rad]
            p.TL     = 0;        % load torque [N*m] (default: no load)
            p.V_max  = 48;       % DC bus voltage limit [V]
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.x0     = [0; 0; 0; 0];
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            id    = x(1);
            iq    = x(2);
            omega = x(3);

            % Voltage inputs (clamped to DC bus limit)
            vd = max(-p.V_max, min(p.V_max, u(1)));
            vq = max(-p.V_max, min(p.V_max, u(2)));

            % Electrical dynamics (dq-frame)
            did_dt = (vd - p.Rs*id + p.pp*omega*p.Lq*iq) / p.Ld;
            diq_dt = (vq - p.Rs*iq - p.pp*omega*(p.Ld*id + p.psi_f)) / p.Lq;

            % Electromagnetic torque
            Te = 1.5 * p.pp * (p.psi_f*iq + (p.Ld - p.Lq)*id*iq);

            % Mechanical dynamics
            domega_dt = (Te - p.B*omega - p.TL) / p.J;
            dtheta_dt = omega;

            xdot = [did_dt; diq_dt; domega_dt; dtheta_dt] + d;
        end

        function y = output(~, x)
            y = [x(3); x(4)];  % omega, theta
        end

        function [e, edot] = get_error(~, x, xref)
            % Track speed (omega)
            e    = xref(3) - x(3);
            edot = xref(4) - x(4);  % approximation via state
        end

        function Te = get_torque(obj, x)
            %GET_TORQUE Compute electromagnetic torque from current state.
            p = obj.params;
            id = x(1);
            iq = x(2);
            Te = 1.5 * p.pp * (p.psi_f*iq + (p.Ld - p.Lq)*id*iq);
        end
    end
end
