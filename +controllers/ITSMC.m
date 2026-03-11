classdef ITSMC < core.Controller
    %ITSMC Integral Terminal Sliding Mode Controller with ELM estimation.
    %
    %   Combines integral terminal sliding surface with RBF-ELM online
    %   disturbance estimation for robust finite-time convergence.
    %
    %   Sliding surface (per channel):
    %       s = e_dot + c1*e + c2 * integral(|e|^(p/q) * sign(e))
    %
    %   Control law:
    %       u = u_eq + u_rbf + u_rob
    %       u_eq  = -f(x) + ddx_d - c1*e_dot - c2*|e|^(p/q)*sign(e)
    %       u_rbf = -d_hat    (RBF-ELM disturbance compensation)
    %       u_rob = -K*sat(s/phi) - lambda_s*s    (robust term)
    %
    %   This controller operates per-channel (SISO). For MIMO systems
    %   (e.g., quadrotor), use CascadedSMC architecture with one ITSMC
    %   per translational axis.
    %
    %   Parameters:
    %       c1       - (scalar) linear surface gain (default 3)
    %       c2       - (scalar) integral surface gain (default 1)
    %       p        - (int) power numerator, odd (default 5)
    %       q        - (int) power denominator, odd, p < q (default 7)
    %       K        - (scalar) switching gain (default 0.5)
    %       lambda_s - (scalar) reaching gain (default 2)
    %       phi      - (scalar) boundary layer width (default 0.2)
    %       u_max    - (scalar) control saturation (default 10)
    %       dt       - (scalar) integration step (default 1e-3)
    %
    %   Reference: Al Ghanimi (2026). "RBF-ELM-ITSMC Hybrid Controller
    %   for Quadrotor UAV Trajectory Tracking." (Manuscript)

    properties
        name = 'ITSMC'
    end

    methods
        function obj = ITSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end
            p.c1       = 3;
            p.c2       = 1;
            p.pp       = 5;       % 'p' conflicts with MATLAB
            p.q        = 7;
            p.K        = 0.5;
            p.lambda_s = 2;
            p.phi      = 0.2;
            p.u_max    = 10;
            p.dt       = 1e-3;
            obj.params = p;
            obj.state  = struct('e_integral', 0, 'initialized', false);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p = obj.params;

            % Get error (uses first channel for SISO operation)
            [e_vec, edot_vec] = plant.get_error(x, xref);
            e    = e_vec(1);
            edot = edot_vec(1);

            % Fractional power integral term
            pq = p.pp / p.q;
            e_pq = abs(e + 1e-10)^pq * sign(e);
            obj.state.e_integral = obj.state.e_integral + e_pq * p.dt;

            % Sliding surface
            s = edot + p.c1 * e + p.c2 * obj.state.e_integral;

            % RBF-ELM disturbance estimation
            if isa(obj.estimator, 'estimators.RBF_ELM')
                % Direct predict using error features
                e_clamp    = max(-3, min(3, e));
                edot_clamp = max(-5, min(5, edot));
                x_est = [e_clamp, edot_clamp];
                d_hat = obj.estimator.predict(x_est);
            else
                y = plant.output(x);
                dh = obj.estimator.estimate(t, x, zeros(plant.n_inputs, 1), y);
                d_hat = dh(1);
            end

            % Saturation function
            if abs(s) <= p.phi
                sat_s = s / p.phi;
            else
                sat_s = sign(s);
            end

            % Control law components
            % u_eq: equivalent control (set f_x = 0 for generic use;
            %        CascadedSMC handles known dynamics externally)
            % Sign convention: with e = xref - x, sdot = -u + c1*edot + c2*e_pq
            u_eq  = p.c1 * edot + p.c2 * e_pq;
            u_rbf = -d_hat;
            u_rob = p.K * sat_s + p.lambda_s * s;

            u_scalar = u_eq + u_rbf + u_rob;

            % Saturate
            u_scalar = max(-p.u_max, min(p.u_max, u_scalar));

            % Online update of RBF-ELM estimator
            if isa(obj.estimator, 'estimators.RBF_ELM') && obj.state.initialized
                d_target = p.K * sat_s;
                obj.estimator.online_update(x_est, d_target);
            end
            obj.state.initialized = true;

            % Format output
            u = u_scalar * ones(plant.n_inputs, 1);
            u = u(1:plant.n_inputs);

            % Diagnostics
            info.s        = s;
            info.ueq      = u_eq;
            info.urbf     = u_rbf;
            info.urob     = u_rob;
            info.dhat     = d_hat;
            info.e        = e;
            info.edot     = edot;
            info.e_pq     = e_pq;
            info.e_int    = obj.state.e_integral;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('e_integral', 0, 'initialized', false);
        end
    end
end
