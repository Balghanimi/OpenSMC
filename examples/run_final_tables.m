%% FINAL: Generate paper table values
%  Table 3: Crane HSMC (M=37.32, transport 0→2m, T=15s)
%  Table 4: Quadrotor ITSMC-RBF-ELM (T=10s, 3D circular)

clear; clc; close all;
addpath('..');

%% ===== TABLE 3: Crane HSMC =====
fprintf('\n===== TABLE 3: CRANE HSMC =====\n');

crane = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
crane.x0 = zeros(4, 1);
ref_fn = utils.references.transport(2.0, 4);
dist_none = utils.disturbances.none(4);
dummy_surf = surfaces.LinearSurface('c', 1);
dummy_reach = reaching.ConstantRate('k', 1);

% Aggregated HSMC (Sec 4.2)
ctrl_agg = controllers.AggregatedHSMC(dummy_surf, dummy_reach);
ctrl_agg.params.c1=0.7; ctrl_agg.params.c2=8.2; ctrl_agg.params.alpha=-2.3;
ctrl_agg.params.kappa=3; ctrl_agg.params.eta=0.1;
arch_agg = architectures.DirectSMC(ctrl_agg);

% Incremental HSMC (Sec 4.3, tuned with control saturation)
ctrl_inc = controllers.IncrementalHSMC(dummy_surf, dummy_reach);
ctrl_inc.params.c1=0.85; ctrl_inc.params.c2=3.6; ctrl_inc.params.c3=0.4;
ctrl_inc.params.kappa=0.3; ctrl_inc.params.eta=0.03;
ctrl_inc.params.u_max = 500;
arch_inc = architectures.DirectSMC(ctrl_inc);

% Combining HSMC (Sec 4.4, tuned for convergence)
ctrl_cmb = controllers.CombiningHSMC(dummy_surf, dummy_reach);
ctrl_cmb.params.c=0.1; ctrl_cmb.params.alpha=0.487;
ctrl_cmb.params.kappa=0.5; ctrl_cmb.params.eta=0.05;
ctrl_cmb.params.u_max = 500;
arch_cmb = architectures.DirectSMC(ctrl_cmb);

runner = benchmark.BenchmarkRunner('dt', 1e-4, 'T', 15);
runner.add_architecture('Aggregated', arch_agg);
runner.add_architecture('Incremental', arch_inc);
runner.add_architecture('Combining', arch_cmb);
runner.add_plant('Crane_Nominal', crane, ref_fn, dist_none);

fprintf('Running crane benchmark...\n');
results = runner.run_all();

names = {'Aggregated', 'Incremental', 'Combining'};
for i = 1:3
    r = results(i,1).result;
    m = benchmark.Metrics.compute_all(r);
    ms = rad2deg(benchmark.Metrics.max_swing(r, 3));
    rs = rad2deg(benchmark.Metrics.residual_swing(r, 3));
    fprintf('CRANE_ROW|%s|%.4f|%.2f|%.1f|%.1f|%.2f|%.4f\n', ...
        names{i}, m.rmse, m.settling_time, m.overshoot, m.control_effort, ms, rs);
end

%% ===== TABLE 4: Quadrotor ITSMC-RBF-ELM =====
fprintf('\n===== TABLE 4: QUADROTOR ITSMC-RBF-ELM =====\n');

dt = 1e-3; T = 10; t = 0:dt:T; N = length(t);
ref_fn_q = utils.references.circular_3d(2.0, 0.5, 1.0, 0.3, 0.2);
ref_x = zeros(1,N); ref_y = zeros(1,N); ref_z = zeros(1,N);
for j = 1:N
    xr = ref_fn_q(t(j));
    ref_x(j) = xr(1); ref_y(j) = xr(2); ref_z(j) = xr(3);
end

rng(42);
d_ext_x = 0.2*sin(2*t) + 0.05*randn(1,N);
d_ext_y = 0.15*cos(3*t) + 0.05*randn(1,N);
d_ext_z = 0.1*sin(t) + 0.03*randn(1,N);

scenarios = {'Nominal', 'Disturbed'};
dist_scales = [0, 1];

for sc = 1:2
    quad = plants.Quadrotor6DOF();
    quad.x0 = zeros(12,1);
    est_x = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);
    est_y = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);
    est_z = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);
    dummy_surf_q = surfaces.IntegralTerminalSurface('c1',5,'c2',2,'p',5,'q',7);
    dummy_reach_q = reaching.Saturation('k',5,'phi',0.1);

    ctrl_x = controllers.ITSMC(dummy_surf_q, dummy_reach_q, est_x);
    ctrl_x.params.c1=2; ctrl_x.params.c2=0.5; ctrl_x.params.K=2; ctrl_x.params.lambda_s=1;
    ctrl_x.params.u_max=8; ctrl_x.params.dt=dt;

    ctrl_y = controllers.ITSMC(dummy_surf_q, dummy_reach_q, est_y);
    ctrl_y.params.c1=2; ctrl_y.params.c2=0.5; ctrl_y.params.K=2; ctrl_y.params.lambda_s=1;
    ctrl_y.params.u_max=8; ctrl_y.params.dt=dt;

    ctrl_z = controllers.ITSMC(dummy_surf_q, dummy_reach_q, est_z);
    ctrl_z.params.c1=2; ctrl_z.params.c2=0.5; ctrl_z.params.K=1.5; ctrl_z.params.lambda_s=0.5;
    ctrl_z.params.u_max=8; ctrl_z.params.dt=dt;

    arch = architectures.CascadedSMC(ctrl_x, ctrl_y, ctrl_z);
    X = zeros(N,12); X(1,:) = quad.x0';
    U_log = zeros(N,4);

    fprintf('Running quadrotor %s...\n', scenarios{sc});
    for k = 1:N-1
        x_k = X(k,:)';
        xref = ref_fn_q(t(k));
        [u_k, ~] = arch.compute(t(k), x_k, xref, quad);
        d_k = zeros(12,1);
        d_k(7) = dist_scales(sc)*d_ext_x(k)/quad.params.m;
        d_k(8) = dist_scales(sc)*d_ext_y(k)/quad.params.m;
        d_k(9) = dist_scales(sc)*d_ext_z(k)/quad.params.m;
        f1 = quad.dynamics(t(k), x_k, u_k, d_k);
        f2 = quad.dynamics(t(k)+dt/2, x_k+dt/2*f1, u_k, d_k);
        f3 = quad.dynamics(t(k)+dt/2, x_k+dt/2*f2, u_k, d_k);
        f4 = quad.dynamics(t(k)+dt, x_k+dt*f3, u_k, d_k);
        X(k+1,:) = (x_k + dt/6*(f1+2*f2+2*f3+f4))';
        U_log(k,:) = u_k';
    end

    ss = round(3/dt);
    ex = ref_x'-X(:,1); ey = ref_y'-X(:,2); ez = ref_z'-X(:,3);
    rmse_total = sqrt(mean(ex(ss:end).^2 + ey(ss:end).^2 + ez(ss:end).^2));
    mae_total  = mean(abs(ex(ss:end)) + abs(ey(ss:end)) + abs(ez(ss:end)));

    % Overshoot: max deviation from reference in radial direction
    r_ref = sqrt(ref_x.^2 + ref_y.^2);
    r_act = sqrt(X(:,1).^2 + X(:,2).^2);
    os = max(0, max(r_act' - r_ref) / max(r_ref) * 100);

    effort = trapz(t, U_log(:,1)'.^2);
    du = diff(U_log(:,1)); chatter = sum(abs(du))/T;

    rmse_x = sqrt(mean(ex(ss:end).^2));
    rmse_y = sqrt(mean(ey(ss:end).^2));
    rmse_z = sqrt(mean(ez(ss:end).^2));

    fprintf('QUAD_ROW|%s|%.4f|%.4f|%.2f|%.1f|%.1f|%.4f,%.4f,%.4f\n', ...
        scenarios{sc}, rmse_total, mae_total, os, effort, chatter, rmse_x, rmse_y, rmse_z);
end

fprintf('\n===== DONE =====\n');
