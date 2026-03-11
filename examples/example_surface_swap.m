%% OpenSMC: Surface Swap Demo
%
%  Shows the key advantage of modular design: swap ONLY the sliding
%  surface while keeping the same reaching law, same plant, same metrics.
%  This is what enables fair benchmarking.

clear; clc; close all;
addpath('..');

%% Shared components
reaching_law = reaching.ExponentialRate('k', 12, 'q', 5);
plant        = plants.DoubleIntegrator('x0', [0; 0]);
ref_fn       = utils.references.sinusoidal(1.0, 0.5, 2);  % sine wave
dist_fn      = utils.disturbances.sinusoidal(0.3, 2.0, 2); % HF disturbance

%% 5 surfaces, SAME reaching law
surfaces_list = {
    'Linear',              surfaces.LinearSurface('c', 10);
    'Terminal',            surfaces.TerminalSurface('beta', 10, 'p', 5, 'q', 7);
    'Nonsingular Terminal', surfaces.NonsingularTerminalSurface('beta', 10, 'p', 7, 'q', 5);
    'Fast Terminal',       surfaces.NonsingularTerminalSurface('beta', 10, 'p', 7, 'q', 5, 'alpha', 0.5);
    'Integral Terminal',   surfaces.IntegralTerminalSurface('c1', 10, 'c2', 5, 'p', 5, 'q', 7);
};

%% Build and benchmark
runner = benchmark.BenchmarkRunner('dt', 1e-4, 'T', 10);

for i = 1:size(surfaces_list, 1)
    ctrl = controllers.ClassicalSMC(surfaces_list{i,2}, reaching_law);
    arch = architectures.DirectSMC(ctrl);
    runner.add_architecture(surfaces_list{i,1}, arch);
end

runner.add_plant('DoubleIntegrator', plant, ref_fn, dist_fn);

%% Run and compare
results = runner.run_all();
runner.print_table(results);
runner.plot_comparison(results, 'DoubleIntegrator');

fprintf('\n--- This is what fair benchmarking looks like. ---\n');
fprintf('Same plant, same reaching law, same disturbance.\n');
fprintf('Only the sliding surface changes.\n');
