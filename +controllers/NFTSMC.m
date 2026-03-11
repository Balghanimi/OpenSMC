classdef NFTSMC < core.Controller
    %NFTSMC Nonsingular Fast Terminal Sliding Mode Controller.
    %
    %   Combines a nonsingular fast terminal sliding surface with a
    %   combined linear+terminal reaching law and optional integral
    %   action for unmodeled dynamics compensation.
    %
    %   Sliding surface:
    %       s = edot + alpha*e + beta * |e|^gamma * sign(e)
    %
    %   Control law:
    %       u_smc = (1/g) * (-f - alpha*edot - ds - k1*s - k2*sig(s)^rho - eta*sn)
    %       u = u_smc + kI * integral(e)
    %
    %   where:
    %       ds = beta*gamma*|e|^(gamma-1)*edot   (surface derivative term)
    %       sn = s / (|s| + delta)               (continuous sign approx)
    %       sig(s)^rho = |s|^rho * sign(s)       (fractional power)
    %       g = input gain (from plant model)
    %       f = approximate error dynamics
    %
    %   Guarantees finite-time convergence with NO singularity.
    %
    %   Parameters:
    %       alpha    - (scalar) linear surface gain (default 500)
    %       beta     - (scalar) terminal surface gain (default 120)
    %       gamma    - (scalar) terminal exponent, 1 < gamma < 2 (default 7/5)
    %       k1       - (scalar) linear reaching gain (default 300)
    %       k2       - (scalar) terminal reaching gain (default 80)
    %       rho      - (scalar) reaching exponent, 0 < rho < 1 (default 5/7)
    %       eta      - (scalar) switching gain for robustness (default 25)
    %       delta    - (scalar) boundary layer for sign approx (default 1e-9)
    %       kI       - (scalar) integral gain for unmodeled dynamics (default 0)
    %       u_max    - (scalar) control saturation (default 20)
    %       dt       - (scalar) integration step (default 1e-5)
    %       input_gain - (scalar) plant input gain g (default -1)
    %       use_feedforward - (logical) use model-based feedforward (default false)
    %       wn       - (scalar) dominant natural frequency for feedforward (default 0)
    %       zeta     - (scalar) dominant damping ratio for feedforward (default 0)
    %       ff_corr  - (scalar) feedforward correction term (default 0)
    %
    %   Reference: Al Ghanimi (2026). "Nonsingular Fast Terminal Sliding
    %   Mode Control for Vibration Suppression in Dual-Stage Nanopositioners."
    %   (Manuscript, Mechatronics)

    properties
        name = 'NFTSMC'
    end

    methods
        function obj = NFTSMC(surface, reaching, estimator)
            obj.surface  = surface;
            obj.reaching = reaching;
            if nargin < 3 || isempty(estimator)
                obj.estimator = estimators.NoEstimator();
            else
                obj.estimator = estimator;
            end

            % Default parameters (tuned for nanopositioner)
            p.alpha   = 500;
            p.beta    = 120;
            p.gamma   = 7/5;     % 1.4
            p.k1      = 300;
            p.k2      = 80;
            p.rho     = 5/7;     % ~0.714
            p.eta     = 25;
            p.delta   = 1e-9;
            p.kI      = 0;       % integral gain (set nonzero for mode compensation)
            p.u_max   = 20;
            p.dt      = 1e-5;
            % Plant model parameters for equivalent control
            p.input_gain     = -1;    % g: set from plant.get_input_gain()
            p.use_feedforward = false;
            p.wn             = 0;     % dominant mode natural frequency
            p.zeta           = 0;     % dominant mode damping ratio
            p.ff_corr        = 0;     % feedforward correction for multi-mode plant

            obj.params = p;
            obj.state  = struct('e_integral', 0, 'e_prev', 0);
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            p = obj.params;

            % Get error
            [e_vec, edot_vec] = plant.get_error(x, xref);
            e    = e_vec(1);
            edot = edot_vec(1);

            % Sliding surface: s = edot + alpha*e + beta*|e|^gamma*sign(e)
            ae = abs(e);
            if ae > 1e-18
                sig_e = ae^p.gamma * sign(e);
            else
                sig_e = 0;
            end
            s = edot + p.alpha * e + p.beta * sig_e;

            % Surface derivative term: d/dt[beta*|e|^gamma*sign(e)]
            if ae > 1e-15
                ds = p.beta * p.gamma * ae^(p.gamma - 1) * edot;
            else
                ds = 0;
            end

            % Terminal reaching: sig(s)^rho = |s|^rho * sign(s)
            as = abs(s);
            if as > 1e-18
                sig_s = as^p.rho * sign(s);
            else
                sig_s = 0;
            end

            % Continuous sign approximation
            sn = s / (as + p.delta);

            % Model-based feedforward (f approximation)
            if p.use_feedforward && p.wn > 0
                yr = xref(1);
                f = (p.wn^2 + p.ff_corr) * yr - p.wn^2 * e - 2*p.zeta*p.wn * edot;
            else
                f = 0;
            end

            % Disturbance estimate
            if isa(obj.estimator, 'estimators.NoEstimator')
                d_hat = 0;
            else
                y = plant.output(x);
                dh = obj.estimator.estimate(t, x, zeros(plant.n_inputs, 1), y);
                d_hat = dh(1);
            end

            % SMC control law
            g = p.input_gain;
            u_smc = (1/g) * (-f - p.alpha*edot - ds ...
                     - p.k1*s - p.k2*sig_s - p.eta*sn) - d_hat;

            % Integral action
            obj.state.e_integral = obj.state.e_integral + p.dt * e;
            u_int = p.kI * obj.state.e_integral;

            u_scalar = u_smc + u_int;

            % Saturate
            u_scalar = max(-p.u_max, min(p.u_max, u_scalar));

            % Store for next step
            obj.state.e_prev = e;

            % Format output
            u = u_scalar * ones(plant.n_inputs, 1);
            u = u(1:plant.n_inputs);

            % Diagnostics
            info.s      = s;
            info.ueq    = (1/g) * (-f - p.alpha*edot - ds);
            info.ur     = (1/g) * (-p.k1*s - p.k2*sig_s - p.eta*sn);
            info.uint   = u_int;
            info.dhat   = d_hat;
            info.e      = e;
            info.edot   = edot;
            info.sig_e  = sig_e;
            info.sig_s  = sig_s;
            info.e_int  = obj.state.e_integral;
        end

        function reset(obj)
            reset@core.Controller(obj);
            obj.state = struct('e_integral', 0, 'e_prev', 0);
        end
    end
end
