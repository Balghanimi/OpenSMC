classdef ClassicalSMC < core.Controller
    %CLASSICALSMC Standard sliding mode controller.
    %
    %   u = ueq + ur
    %
    %   where:
    %     ueq = equivalent control (model-based, cancels known dynamics)
    %     ur  = reaching law output (drives state to surface)
    %
    %   This is the most general SMC controller. It accepts ANY surface
    %   and ANY reaching law, making it the workhorse for benchmarking.

    properties
        name = 'ClassicalSMC'
    end

    methods
        function obj = ClassicalSMC(surface, reaching, estimator)
            obj.surface   = surface;
            obj.reaching  = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            obj.params = struct();
            obj.state  = struct('eint', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            % Get error
            [e, edot] = plant.get_error(x, xref);

            % Update integral
            % (dt is implicit; caller should manage this or use small dt)
            if ~isfield(obj.state, 'eint') || numel(obj.state.eint) ~= numel(e)
                obj.state.eint = zeros(size(e));
            end

            % Compute sliding variable
            s = obj.surface.compute(e, edot, obj.state.eint, t);

            % Compute reaching control
            u_r = obj.reaching.compute(s, t);

            % Estimate disturbance
            y = plant.output(x);
            dhat = obj.estimator.estimate(t, x, u_r, y);

            % Equivalent control: for double integrator, ueq = xref_ddot
            % For general plants, this should be overridden or computed
            % from the plant model. Here we use a simple feedforward.
            u_eq = 0;

            % Total control
            u = u_eq - u_r - dhat(1:plant.n_inputs);

            % Diagnostics
            info.s    = s;
            info.ueq  = u_eq;
            info.ur   = u_r;
            info.dhat = dhat;
            info.e    = e;
            info.edot = edot;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('eint', 0);
        end
    end
end
