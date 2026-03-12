%% Example: Predefined-Time vs Global vs Linear Surface Comparison
%
%  Demonstrates time-guarantee properties of different surface designs
%  on a double integrator plant.
%
%  - LinearSurface: asymptotic convergence only
%  - GlobalSurface: sliding from t=0, still asymptotic on surface
%  - PredefinedTimeSurface: guaranteed convergence before Tc
%
%  Mathematical formulations from:
%    - Sanchez-Torres et al. (2018), "Predefined-time stability"
%    - Bartoszewicz (1998), "Discrete-time quasi-sliding-mode"

clear; clc; close all;

%% 1. Plant
plant = plants.DoubleIntegrator();
plant.x0 = [5; 0];  % large initial displacement

%% 2. Reference and disturbance
ref_fn  = @(t) [0; 0];
dist_fn = @(t) [0; 0.5*sin(2*t)];  % sinusoidal disturbance

%% 3. Three surface designs
Tc = 1.5;  % predefined convergence time

surf1 = surfaces.LinearSurface('c', 10);
surf2 = surfaces.GlobalSurface('c', 10, 'alpha', 5);
surf3 = surfaces.PredefinedTimeSurface('Tc', Tc, 'c_inf', 10);

reach = reaching.SuperTwisting('k1', 15, 'k2', 10);

ctrl1 = controllers.ClassicalSMC(surf1, reach);
ctrl2 = controllers.ClassicalSMC(surf2, reach);
ctrl3 = controllers.ClassicalSMC(surf3, reach);

arch1 = architectures.DirectSMC(ctrl1);
arch2 = architectures.DirectSMC(ctrl2);
arch3 = architectures.DirectSMC(ctrl3);

%% 4. Simulate
sim = benchmark.Simulator('dt', 1e-4, 'T', 4);

r1 = sim.run(arch1, plant, ref_fn, dist_fn);
r2 = sim.run(arch2, plant, ref_fn, dist_fn);
r3 = sim.run(arch3, plant, ref_fn, dist_fn);

%% 5. Results
fprintf('\n=== Surface Design Comparison ===\n\n');
names = {'Linear', 'Global', 'Predefined-Time'};
results = {r1, r2, r3};

fprintf('%-18s  %10s  %10s  %10s\n', 'Surface', 'RMSE', 'Settling', 'ISE');
fprintf('%s\n', repmat('-', 1, 52));

for i = 1:3
    rmse = benchmark.Metrics.rmse(results{i});
    ts   = benchmark.Metrics.settling_time(results{i});
    ise  = benchmark.Metrics.ise(results{i});
    fprintf('%-18s  %10.6f  %10.4f  %10.6f\n', names{i}, rmse, ts, ise);
end

fprintf('\nPredefined convergence time Tc = %.1f s\n', Tc);

%% 6. Plot
figure('Name', 'Surface Comparison', 'Position', [100 100 900 500]);

subplot(2,2,1);
plot(r1.t, r1.x(1,:), 'b', r2.t, r2.x(1,:), 'r', ...
     r3.t, r3.x(1,:), 'g', 'LineWidth', 1.2);
xline(Tc, 'k--', 'T_c', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Position');
title('State x_1'); legend(names{:});

subplot(2,2,2);
plot(r1.t, r1.e(1,:), 'b', r2.t, r2.e(1,:), 'r', ...
     r3.t, r3.e(1,:), 'g', 'LineWidth', 1.2);
xline(Tc, 'k--', 'T_c', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Error');
title('Tracking Error'); legend(names{:});

subplot(2,2,3);
plot(r1.t, r1.s(1,:), 'b', r2.t, r2.s(1,:), 'r', ...
     r3.t, r3.s(1,:), 'g', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('s');
title('Sliding Variable'); legend(names{:});

subplot(2,2,4);
plot(r1.t, r1.u(1,:), 'b', r2.t, r2.u(1,:), 'r', ...
     r3.t, r3.u(1,:), 'g', 'LineWidth', 0.8);
xlabel('Time [s]'); ylabel('u');
title('Control Input'); legend(names{:});

sgtitle('Linear vs Global vs Predefined-Time Sliding Surfaces');
