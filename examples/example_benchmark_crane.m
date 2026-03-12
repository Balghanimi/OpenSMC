%% OpenSMC Benchmark: Compare HSMC Methods on Overhead Crane
%
%  Uses the BenchmarkRunner framework to systematically compare
%  three hierarchical SMC variants from Qian & Yi (2015):
%    1. Aggregated HSMC  (two-layer, Sec 4.2)
%    2. Incremental HSMC (three-layer, Sec 4.3)
%    3. Combining HSMC   (intermediate variable, Sec 4.4)
%
%  Scenarios:
%    A. Nominal transport (0 → 2m, no disturbance)
%    B. With sinusoidal disturbance
%
%  Run this script from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% 1. Create Plant
crane = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
crane.x0 = zeros(4, 1);

%% 2. Reference and Disturbance Functions
ref_fn   = utils.references.transport(2.0, 4);
dist_none = utils.disturbances.none(4);
dist_sin  = utils.disturbances.sinusoidal(5.0, 0.5, 4);

%% 3. Create Architectures (HSMC controllers wrapped in DirectSMC)
dummy_surf  = surfaces.LinearSurface('c', 1);
dummy_reach = reaching.ConstantRate('k', 1);

% Aggregated HSMC
ctrl_agg = controllers.AggregatedHSMC(dummy_surf, dummy_reach);
ctrl_agg.params.c1    = 0.7;
ctrl_agg.params.c2    = 8.2;
ctrl_agg.params.alpha = -2.3;
ctrl_agg.params.kappa = 3;
ctrl_agg.params.eta   = 0.1;
arch_agg = architectures.DirectSMC(ctrl_agg);

% Incremental HSMC
% Note: den = c3*b2+b1 ≈ 0.017 for this crane, so kappa is amplified ~60x.
% Reduce kappa to prevent excessive initial force and overshoot.
ctrl_inc = controllers.IncrementalHSMC(dummy_surf, dummy_reach);
ctrl_inc.params.c1    = 0.85;
ctrl_inc.params.c2    = 3.6;
ctrl_inc.params.c3    = 0.4;
ctrl_inc.params.kappa = 0.3;
ctrl_inc.params.eta   = 0.02;
arch_inc = architectures.DirectSMC(ctrl_inc);

% Combining HSMC
% Note: den = b1+c*b2 ≈ 0.021 for this crane, so kappa is amplified ~48x.
ctrl_cmb = controllers.CombiningHSMC(dummy_surf, dummy_reach);
ctrl_cmb.params.c     = 0.242;
ctrl_cmb.params.alpha = 0.487;
ctrl_cmb.params.kappa = 1.0;
ctrl_cmb.params.eta   = 0.05;
arch_cmb = architectures.DirectSMC(ctrl_cmb);

%% 4. Run Benchmark
runner = benchmark.BenchmarkRunner('dt', 1e-4, 'T', 8);

runner.add_architecture('AggregatedHSMC',  arch_agg);
runner.add_architecture('IncrementalHSMC', arch_inc);
runner.add_architecture('CombiningHSMC',   arch_cmb);

runner.add_plant('Crane_Nominal',   crane, ref_fn, dist_none);
runner.add_plant('Crane_Disturbed', crane, ref_fn, dist_sin);

fprintf('Starting HSMC benchmark (3 controllers x 2 scenarios)...\n');
results = runner.run_all();

%% 5. Print Results Table
runner.print_table(results);

%% 6. Print Underactuated Metrics
fprintf('\n=== Underactuated Metrics ===\n');
[na, np] = size(results);
for j = 1:np
    fprintf('\n--- %s ---\n', results(1,j).plant_name);
    fprintf('%-20s %12s %15s\n', 'Controller', 'MaxSwing(deg)', 'ResidSwing(deg)');
    fprintf('%s\n', repmat('-', 1, 49));
    for i = 1:na
        ms  = rad2deg(benchmark.Metrics.max_swing(results(i,j).result, 3));
        rs  = rad2deg(benchmark.Metrics.residual_swing(results(i,j).result, 3));
        fprintf('%-20s %12.3f %15.5f\n', results(i,j).arch_name, ms, rs);
    end
end

%% 7. Custom Comparison Plot
runner.plot_comparison(results, 'Crane_Nominal');

% Additional underactuated plot
figure('Name', 'Crane: Swing Angle Comparison', 'Position', [100 100 1000 500]);

for j = 1:np
    subplot(1, np, j); hold on;
    for i = 1:na
        plot(results(i,j).result.t, ...
            rad2deg(results(i,j).result.x(3,:)), 'LineWidth', 1.2, ...
            'DisplayName', results(i,j).arch_name);
    end
    xlabel('Time (s)'); ylabel('\theta (deg)');
    title(['Payload Swing: ' results(1,j).plant_name], 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
end

sgtitle('HSMC Swing Angle Comparison (OpenSMC Benchmark)');

fprintf('\nBenchmark complete.\n');
