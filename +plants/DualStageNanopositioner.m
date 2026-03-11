classdef DualStageNanopositioner < core.Plant
    %DUALSTAGENANOPOSITIONER Piezoelectric fine stage with two resonant modes.
    %
    %   Two-mode vibratory plant model for precision positioning.
    %   Models the piezoelectric fine stage of a dual-stage nanopositioner
    %   (VCM coarse + piezo fine) used in hard-disk drives and nanofabrication.
    %
    %   Transfer function (per mode):
    %       G_k(s) = Kpk * wnk^2 / (s^2 + 2*zk*wnk*s + wnk^2)
    %
    %   State-space:
    %       x = [x1; x1dot; x2; x2dot]   (mode1 pos, vel; mode2 pos, vel)
    %       A = [0 1 0 0; -wn1^2 -2*z1*wn1 0 0; 0 0 0 1; 0 0 -wn2^2 -2*z2*wn2]
    %       B = [0; Kp1*wn1^2; 0; Kp2*wn2^2]
    %       C = [1 0 1 0]   (output = sum of modes)
    %
    %   Default parameters from Fleming (2009) — a typical piezoelectric
    %   flexure-guided nanopositioner:
    %       Mode 1: fn1 = 780 Hz, z1 = 0.012, Kp1 = 0.145 um/V
    %       Mode 2: fn2 = 2340 Hz, z2 = 0.008, Kp2 = 0.015 um/V
    %
    %   Reference: Al Ghanimi (2026). "Nonsingular Fast Terminal Sliding
    %   Mode Control for Vibration Suppression in Dual-Stage Nanopositioners."
    %   (Manuscript, Mechatronics)

    properties
        name             = 'DualStageNanopositioner'
        n_states         = 4
        n_inputs         = 1
        n_outputs        = 1
        n_dof            = 1
        is_underactuated = false
    end

    methods
        function obj = DualStageNanopositioner(varargin)
            % Mode 1 (dominant)
            p.Kp1 = 0.145e-6;   % DC gain [m/V]
            p.fn1 = 780;         % natural frequency [Hz]
            p.z1  = 0.012;       % damping ratio [-]
            % Mode 2 (secondary)
            p.Kp2 = 0.015e-6;   % DC gain [m/V]
            p.fn2 = 2340;        % natural frequency [Hz]
            p.z2  = 0.008;       % damping ratio [-]
            % Amplifier limit
            p.u_max = 20;        % voltage limit [V]

            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end

            % Derived angular frequencies
            p.wn1 = 2 * pi * p.fn1;
            p.wn2 = 2 * pi * p.fn2;

            obj.params = p;
            obj.x0 = zeros(4, 1);
        end

        function xdot = dynamics(obj, ~, x, u, d)
            p = obj.params;
            % Clamp input voltage
            u = max(-p.u_max, min(p.u_max, u));

            xdot = [
                x(2);
                -p.wn1^2 * x(1) - 2*p.z1*p.wn1 * x(2) + p.Kp1*p.wn1^2 * u;
                x(4);
                -p.wn2^2 * x(3) - 2*p.z2*p.wn2 * x(4) + p.Kp2*p.wn2^2 * u
            ] + d;
        end

        function y = output(obj, x)
            % Total displacement = sum of both modes
            y = x(1) + x(3);
        end

        function [e, edot] = get_error(obj, x, xref)
            % Position error (scalar)
            y = x(1) + x(3);
            ydot = x(2) + x(4);
            e    = xref(1) - y;
            edot = xref(2) - ydot;
        end

        function g = get_input_gain(obj)
            %GET_INPUT_GAIN Total input gain for control law design.
            %   g = -(Kp1*wn1^2 + Kp2*wn2^2)
            %   Negative because: more voltage -> more displacement -> less error.
            p = obj.params;
            g = -(p.Kp1 * p.wn1^2 + p.Kp2 * p.wn2^2);
        end

        function [A, B, C] = get_ss_matrices(obj)
            %GET_SS_MATRICES Return state-space matrices for analysis.
            p = obj.params;
            A = [0, 1, 0, 0;
                 -p.wn1^2, -2*p.z1*p.wn1, 0, 0;
                 0, 0, 0, 1;
                 0, 0, -p.wn2^2, -2*p.z2*p.wn2];
            B = [0; p.Kp1*p.wn1^2; 0; p.Kp2*p.wn2^2];
            C = [1, 0, 1, 0];
        end
    end
end
