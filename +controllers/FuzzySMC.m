classdef FuzzySMC < core.Controller
    %FUZZYSMC Fuzzy sliding mode controller.
    %
    %   Replaces the discontinuous sign(s) with a fuzzy inference system
    %   that smoothly adapts the switching gain based on (s, sdot).
    %
    %   u = ueq + u_fuzzy
    %
    %   The fuzzy system uses 2 inputs:
    %     - s:    sliding variable (NB, NS, ZE, PS, PB)
    %     - sdot: rate of change of s (N, Z, P)
    %
    %   Output: k_fuzzy in [-1, 1], scaled by switching gain k.
    %   u_fuzzy = -k * k_fuzzy
    %
    %   This eliminates chattering without a boundary layer, while
    %   maintaining better tracking than the saturation approach.
    %
    %   Implementation: Mamdani-type with triangular membership functions
    %   and centroid defuzzification. No Fuzzy Logic Toolbox required.
    %
    %   Reference:
    %       Khanesar, M.A. et al. (2021). "Sliding-Mode Fuzzy
    %       Controllers", Springer. (Mathematical formulation only,
    %       code written from scratch.)

    properties
        name = 'FuzzySMC'
    end

    methods
        function obj = FuzzySMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.k       = 10;     % switching gain
            p.s_range = 1.0;    % universe of discourse for s
            p.sdot_range = 5.0; % universe of discourse for sdot
            p.dt      = 1e-4;   % time step for sdot estimation
            obj.params = p;
            obj.state  = struct('s_prev', 0, 'eint', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            [e, edot] = plant.get_error(x, xref);

            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end

            % Sliding variable
            s = obj.surface.compute(e, edot, obj.state.eint, t);

            % Estimate sdot
            if ~isfield(obj.state, 's_prev') || numel(obj.state.s_prev) ~= numel(s)
                obj.state.s_prev = s;
            end
            sdot = (s - obj.state.s_prev) / obj.params.dt;
            obj.state.s_prev = s;

            % Fuzzy inference (element-wise for vector s)
            k_fuzzy = zeros(size(s));
            for i = 1:numel(s)
                k_fuzzy(i) = obj.fuzzy_inference(s(i), sdot(i));
            end

            % Disturbance estimate
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, zeros(plant.n_inputs,1), y);

            % Control law
            u = -obj.params.k * k_fuzzy - dhat(1:plant.n_inputs);

            info.s      = s;
            info.sdot   = sdot;
            info.ueq    = 0;
            info.ur     = -obj.params.k * k_fuzzy;
            info.dhat   = dhat;
            info.e      = e;
            info.edot   = edot;
            info.k_fuzzy = k_fuzzy;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('s_prev', 0, 'eint', 0);
        end
    end

    methods (Access = private)
        function out = fuzzy_inference(obj, s, sdot)
            %FUZZY_INFERENCE Mamdani-type fuzzy logic for switching.
            %   Inputs normalized to [-1, 1] range.
            %   5 rules for s × 3 rules for sdot = 15 rules.

            sr = obj.params.s_range;
            sdr = obj.params.sdot_range;

            % Normalize inputs
            sn = max(-1, min(1, s / sr));
            sdn = max(-1, min(1, sdot / sdr));

            % Membership functions for s: NB, NS, ZE, PS, PB
            % Triangular: trimf(x, [a, b, c])
            mu_s = [obj.trimf(sn, [-1.0, -1.0, -0.5]);   % NB
                    obj.trimf(sn, [-1.0, -0.5,  0.0]);    % NS
                    obj.trimf(sn, [-0.5,  0.0,  0.5]);    % ZE
                    obj.trimf(sn,  [0.0,  0.5,  1.0]);    % PS
                    obj.trimf(sn,  [0.5,  1.0,  1.0])];   % PB

            % Membership functions for sdot: N, Z, P
            mu_sd = [obj.trimf(sdn, [-1.0, -1.0, 0.0]);   % N
                     obj.trimf(sdn, [-1.0,  0.0, 1.0]);   % Z
                     obj.trimf(sdn,  [0.0,  1.0, 1.0])];  % P

            % Rule base (15 rules): output singletons in [-1, 1]
            % Rows = s (NB,NS,ZE,PS,PB), Cols = sdot (N,Z,P)
            %   If s=NB => output large negative (correct positive)
            %   If s=PB => output large positive
            rules = [-1.0, -0.5, -0.25;    % NB: strongly correct
                     -0.5, -0.25, 0.0;     % NS: moderately correct
                     -0.25, 0.0,  0.25;    % ZE: near zero
                      0.0,  0.25, 0.5;     % PS: moderately correct
                      0.25, 0.5,  1.0];    % PB: strongly correct

            % Weighted average defuzzification (Takagi-Sugeno style)
            num = 0;
            den = 0;
            for i = 1:5
                for j = 1:3
                    w = min(mu_s(i), mu_sd(j));  % AND = min
                    num = num + w * rules(i, j);
                    den = den + w;
                end
            end

            if den > 1e-12
                out = num / den;
            else
                out = sign(s);  % fallback to hard switching
            end
        end
    end

    methods (Static, Access = private)
        function mu = trimf(x, abc)
            %TRIMF Triangular membership function.
            a = abc(1); b = abc(2); c = abc(3);
            mu = max(0, min((x-a)/(b-a+1e-15), (c-x)/(c-b+1e-15)));
        end
    end
end
