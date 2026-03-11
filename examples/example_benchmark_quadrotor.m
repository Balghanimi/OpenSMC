%% OpenSMC Benchmark: ITSMC-RBF-ELM on Quadrotor
%
%  Uses the BenchmarkRunner framework to evaluate the ITSMC controller
%  with RBF-ELM disturbance estimation on a 6-DOF quadrotor.
%
%  Scenarios:
%    A. Nominal circular trajectory (no disturbance)
%    B. Circular trajectory with harmonic disturbances
%
%  Architecture: CascadedSMC
%    - Outer loop: 3x ITSMC (one per translational axis) + RBF-ELM
%    - Control allocation: desired acceleration -> thrust + desired angles
%    - Inner loop: PD attitude control
%
%  Run this script from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% 1. Create Plants
quad_nom  = plants.Quadrotor6DOF();
quad_nom.x0 = zeros(12, 1);

quad_dist = plants.Quadrotor6DOF();
quad_dist.x0 = zeros(12, 1);

%% 2. Reference and Disturbance Functions
ref_fn = utils.references.circular_3d(2.0, 0.5, 1.0, 0.3, 0.2);

dist_none = utils.disturbances.none(12);
dist_harm = utils.disturbances.composite({
    utils.disturbances.sinusoidal(0.15, 0.3, 12), ...
    utils.disturbances.random_band(0.03, 42, 12)
});

%% 3. Create ITSMC + CascadedSMC Architecture
dt = 1e-3;

dummy_surf  = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1, 'p', 5, 'q', 7);
dummy_reach = reaching.Saturation('k', 0.5, 'phi', 0.2);

% Per-axis RBF-ELM estimators
est_x = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);
est_y = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);
est_z = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);

% Per-axis ITSMC controllers
ctrl_x = controllers.ITSMC(dummy_surf, dummy_reach, est_x);
ctrl_x.params.c1 = 3;  ctrl_x.params.c2 = 1;
ctrl_x.params.K = 0.5; ctrl_x.params.lambda_s = 2;
ctrl_x.params.dt = dt;

ctrl_y = controllers.ITSMC(dummy_surf, dummy_reach, est_y);
ctrl_y.params.c1 = 3;  ctrl_y.params.c2 = 1;
ctrl_y.params.K = 0.5; ctrl_y.params.lambda_s = 2;
ctrl_y.params.dt = dt;

ctrl_z = controllers.ITSMC(dummy_surf, dummy_reach, est_z);
ctrl_z.params.c1 = 4;  ctrl_z.params.c2 = 2;
ctrl_z.params.K = 1.0; ctrl_z.params.lambda_s = 2;
ctrl_z.params.dt = dt;

arch = architectures.CascadedSMC(ctrl_x, ctrl_y, ctrl_z);

%% 4. Run Benchmark
runner = benchmark.BenchmarkRunner('dt', dt, 'T', 10);

runner.add_architecture('ITSMC-RBF-ELM', arch);

runner.add_plant('Quad_Nominal',   quad_nom,  ref_fn, dist_none);
runner.add_plant('Quad_Disturbed', quad_dist, ref_fn, dist_harm);

fprintf('Starting ITSMC quadrotor benchmark (2 scenarios)...\n');
results = runner.run_all();

%% 5. Print Results Table
runner.print_table(results);

%% 6. Print Per-Axis RMSE (steady-state, t > 3s)
fprintf('\n=== Per-Axis RMSE (t > 3s) ===\n');
[na, np] = size(results);
for j = 1:np
    fprintf('\n--- %s ---\n', results(1,j).plant_name);
    for i = 1:na
        res = results(i,j).result;
        ss = find(res.t >= 3, 1);
        ex = res.xref(1,ss:end) - res.x(1,ss:end);
        ey = res.xref(2,ss:end) - res.x(2,ss:end);
        ez = res.xref(3,ss:end) - res.x(3,ss:end);
        fprintf('%-20s  X: %.4f  Y: %.4f  Z: %.4f m\n', ...
            results(i,j).arch_name, ...
            sqrt(mean(ex.^2)), sqrt(mean(ey.^2)), sqrt(mean(ez.^2)));
    end
end

%% 7. Custom Quadrotor Plots
figure('Name', 'ITSMC Quadrotor Benchmark', 'Position', [50 50 1400 800]);

for j = 1:np
    res = results(1,j).result;

    % 3D trajectory
    subplot(2, np, j);
    plot3(res.xref(1,:), res.xref(2,:), res.xref(3,:), 'r--', 'LineWidth', 1.5); hold on;
    plot3(res.x(1,:), res.x(2,:), res.x(3,:), 'b-', 'LineWidth', 1);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(results(1,j).plant_name, 'Interpreter', 'none');
    legend('Reference', 'Actual'); grid on; view(30, 25);

    % Per-axis tracking errors
    subplot(2, np, np + j);
    plot(res.t, res.e(1,:), res.t, res.e(2,:), res.t, res.e(3,:), 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Tracking Errors'); legend('e_x', 'e_y', 'e_z'); grid on;
end

sgtitle('ITSMC + RBF-ELM Quadrotor Benchmark (OpenSMC)', 'FontSize', 14);

fprintf('\nBenchmark complete.\n');
