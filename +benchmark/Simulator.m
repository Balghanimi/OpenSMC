classdef Simulator < handle
    %SIMULATOR Unified simulation engine for all SMC experiments.
    %
    %   Runs a closed-loop simulation with any Architecture + Plant combo.
    %   Uses fixed-step RK4 by default (consistent across all experiments).
    %
    %   Usage:
    %       sim = benchmark.Simulator('dt', 1e-4, 'T', 10);
    %       result = sim.run(architecture, plant, ref_generator, disturbance);

    properties
        dt      = 1e-4   % (scalar) integration time step [s]
        T       = 10     % (scalar) simulation duration [s]
        solver  = 'rk4'  % (string) 'rk4' | 'euler' | 'ode45'
    end

    methods
        function obj = Simulator(varargin)
            %SIMULATOR Constructor with name-value pairs.
            for i = 1:2:numel(varargin)
                obj.(varargin{i}) = varargin{i+1};
            end
        end

        function result = run(obj, arch, plant, ref_fn, dist_fn)
            %RUN Execute closed-loop simulation.
            %
            %   Inputs:
            %       arch    - (core.Architecture) control architecture
            %       plant   - (core.Plant) plant model
            %       ref_fn  - (function_handle) @(t) -> xref vector
            %       dist_fn - (function_handle) @(t) -> d disturbance vector
            %
            %   Output:
            %       result  - (struct) with fields:
            %           .t     - (1 x N) time vector
            %           .x     - (n x N) state trajectory
            %           .u     - (m x N) control input history
            %           .s     - (p x N) sliding variable history
            %           .xref  - (n x N) reference trajectory
            %           .e     - (n x N) tracking error
            %           .d     - (n x N) disturbance history
            %           .dhat  - (n x N) estimated disturbance
            %           .info  - (struct) architecture/controller info

            N = round(obj.T / obj.dt) + 1;
            t_vec = linspace(0, obj.T, N);

            % Pre-allocate
            n = plant.n_states;
            m = plant.n_inputs;
            x_hist    = zeros(n, N);
            u_hist    = zeros(m, N);
            xref_hist = zeros(n, N);
            d_hist    = zeros(n, N);

            % Storage for sliding variable and diagnostics
            s_hist    = [];  % sized on first iteration
            dhat_hist = [];

            % Initial conditions
            arch.reset();
            x = plant.x0;
            x_hist(:,1) = x;

            for k = 1:N-1
                t = t_vec(k);
                xref = ref_fn(t);
                d    = dist_fn(t);

                xref_hist(:,k) = xref;
                d_hist(:,k)    = d;

                % Controller
                [u, ctrl_info] = arch.compute(t, x, xref, plant);
                u_hist(:,k) = u;

                % Log sliding variable
                if isfield(ctrl_info, 's')
                    if isempty(s_hist)
                        s_hist = zeros(numel(ctrl_info.s), N);
                    end
                    s_hist(:,k) = ctrl_info.s;
                end
                if isfield(ctrl_info, 'dhat')
                    if isempty(dhat_hist)
                        dhat_hist = zeros(numel(ctrl_info.dhat), N);
                    end
                    dhat_hist(:,k) = ctrl_info.dhat;
                end

                % Integration (RK4)
                x = obj.step(plant, t, x, u, d);
                x_hist(:,k+1) = x;
            end

            % Final step logging
            xref_hist(:,N) = ref_fn(t_vec(N));
            d_hist(:,N)    = dist_fn(t_vec(N));
            [u, ctrl_info] = arch.compute(t_vec(N), x, xref_hist(:,N), plant);
            u_hist(:,N) = u;
            if ~isempty(s_hist) && isfield(ctrl_info, 's')
                s_hist(:,N) = ctrl_info.s;
            end

            % Compute error
            e_hist = xref_hist - x_hist;

            % Pack result
            result.t     = t_vec;
            result.x     = x_hist;
            result.u     = u_hist;
            result.s     = s_hist;
            result.xref  = xref_hist;
            result.e     = e_hist;
            result.d     = d_hist;
            result.dhat  = dhat_hist;
            result.info  = arch.describe();
            result.dt    = obj.dt;
        end
    end

    methods (Access = private)
        function x_next = step(obj, plant, t, x, u, d)
            %STEP Single integration step.
            dt_ = obj.dt;
            switch obj.solver
                case 'rk4'
                    k1 = plant.dynamics(t,           x,              u, d);
                    k2 = plant.dynamics(t + dt_/2,   x + dt_/2 * k1, u, d);
                    k3 = plant.dynamics(t + dt_/2,   x + dt_/2 * k2, u, d);
                    k4 = plant.dynamics(t + dt_,     x + dt_   * k3, u, d);
                    x_next = x + (dt_/6) * (k1 + 2*k2 + 2*k3 + k4);
                case 'euler'
                    x_next = x + dt_ * plant.dynamics(t, x, u, d);
                otherwise
                    error('OpenSMC:Simulator', 'Unknown solver: %s', obj.solver);
            end
        end
    end
end
