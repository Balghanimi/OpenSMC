%% OpenSMC Quick Start: Compare 4 SMC Variants on Double Integrator
%
%  This example demonstrates the core workflow:
%    1. Pick surfaces and reaching laws (mix and match)
%    2. Wrap them in controllers and architectures
%    3. Run the benchmark
%    4. Compare with standardized metrics
%
%  Run this script from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% Plant
plant = plants.DoubleIntegrator('x0', [0; 0]);

%% Reference and Disturbance
ref_fn  = utils.references.step_ref(1.0, 2);         % step to 1.0
dist_fn = utils.disturbances.step(0.5, 3.0, 2);      % disturbance at t=3

%% Controller 1: Classical SMC (Linear surface + Sign)
ctrl1 = controllers.ClassicalSMC( ...
    surfaces.LinearSurface('c', 10), ...
    reaching.ConstantRate('k', 15));
arch1 = architectures.DirectSMC(ctrl1);

%% Controller 2: Classical SMC (Linear surface + Saturation)
ctrl2 = controllers.ClassicalSMC( ...
    surfaces.LinearSurface('c', 10), ...
    reaching.Saturation('k', 15, 'phi', 0.1));
arch2 = architectures.DirectSMC(ctrl2);

%% Controller 3: Terminal SMC (Terminal surface + Power rate)
ctrl3 = controllers.ClassicalSMC( ...
    surfaces.TerminalSurface('beta', 10, 'p', 5, 'q', 7), ...
    reaching.PowerRate('k', 15, 'alpha', 0.5));
arch3 = architectures.DirectSMC(ctrl3);

%% Controller 4: Nonsingular FTSMC (NFTSMC surface + Super-twisting)
ctrl4 = controllers.ClassicalSMC( ...
    surfaces.NonsingularTerminalSurface('beta', 10, 'p', 7, 'q', 5), ...
    reaching.SuperTwisting('k1', 15, 'k2', 10));
arch4 = architectures.DirectSMC(ctrl4);

%% Run Benchmark
runner = benchmark.BenchmarkRunner('dt', 1e-4, 'T', 5);

runner.add_architecture('Classical-Sign',    arch1);
runner.add_architecture('Classical-Sat',     arch2);
runner.add_architecture('Terminal-Power',    arch3);
runner.add_architecture('NFTSMC-SuperTwist', arch4);

runner.add_plant('DoubleIntegrator', plant, ref_fn, dist_fn);

results = runner.run_all();

%% Print Comparison Table
runner.print_table(results);

%% Plot
runner.plot_comparison(results, 'DoubleIntegrator');
