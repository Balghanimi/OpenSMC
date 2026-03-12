%% Example: 2-DOF Robot Arm — Multi-Controller Benchmark
%
%  Compares Classical SMC, Fuzzy SMC, and Fixed-Time SMC on a
%  2-link planar robot manipulator tracking a joint trajectory.
%
%  All code written from scratch. Mathematical formulations from:
%    - Slotine & Li (1991), "Applied Nonlinear Control"
%    - Polyakov (2012), "Nonlinear feedback design for fixed-time stabilization"
%    - Khanesar et al. (2021), "Sliding-Mode Fuzzy Controllers"

clear; clc; close all;

%% 1. Create plant
plant = plants.TwoLinkArm('m1', 1, 'm2', 1, 'l1', 1, 'l2', 1);
plant.x0 = [0; 0; 0; 0];

%% 2. Reference: move both joints to pi/4
ref_fn = @(t) [pi/4; 0; pi/4; 0];

%% 3. Disturbance: step torque disturbance at t=2
dist_fn = @(t) [0; (t > 2)*0.5; 0; (t > 2)*0.3];

%% 4. Define controllers
% Surface and reaching law (shared)
surf_lin  = surfaces.LinearSurface('c', 15);
surf_glob = surfaces.GlobalSurface('c', 15, 'alpha', 5);
reach_sat = reaching.Saturation('k', 20, 'phi', 0.1);

% Controller 1: Classical SMC with linear surface
ctrl1 = controllers.ClassicalSMC(surf_lin, reach_sat);
arch1 = architectures.DirectSMC(ctrl1);

% Controller 2: Fuzzy SMC with global surface (chattering-free)
ctrl2 = controllers.FuzzySMC(surf_glob, reach_sat);
arch2 = architectures.DirectSMC(ctrl2);

% Controller 3: Fixed-Time SMC (convergence guaranteed)
ctrl3 = controllers.FixedTimeSMC(surf_lin, reach_sat);
ctrl3.params.alpha = 15;
ctrl3.params.beta  = 15;
ctrl3.params.p = 0.5;
ctrl3.params.q = 1.5;
arch3 = architectures.DirectSMC(ctrl3);

%% 5. Simulate all
sim = benchmark.Simulator('dt', 1e-4, 'T', 5);

fprintf('Running Classical SMC...\n');
result1 = sim.run(arch1, plant, ref_fn, dist_fn);

fprintf('Running Fuzzy SMC...\n');
result2 = sim.run(arch2, plant, ref_fn, dist_fn);

fprintf('Running Fixed-Time SMC...\n');
result3 = sim.run(arch3, plant, ref_fn, dist_fn);

%% 6. Compare metrics
fprintf('\n=== Robot Arm Benchmark Results ===\n\n');
names = {'Classical SMC', 'Fuzzy SMC', 'Fixed-Time SMC'};
results = {result1, result2, result3};

fprintf('%-18s  %10s  %10s  %10s  %12s\n', ...
    'Controller', 'RMSE', 'ISE', 'Settling', 'Chattering');
fprintf('%s\n', repmat('-', 1, 65));

for i = 1:3
    r = results{i};
    rmse = benchmark.Metrics.rmse(r);
    ise  = benchmark.Metrics.ise(r);
    ts   = benchmark.Metrics.settling_time(r);
    ci   = benchmark.Metrics.chattering_index(r);
    fprintf('%-18s  %10.6f  %10.6f  %10.4f  %12.4f\n', ...
        names{i}, rmse, ise, ts, ci);
end

%% 7. Plot comparison
figure('Name', 'Robot Arm Benchmark', 'Position', [100 100 1000 600]);

% Joint 1
subplot(2,2,1);
plot(result1.t, result1.x(1,:), 'b', ...
     result2.t, result2.x(1,:), 'r', ...
     result3.t, result3.x(1,:), 'g', ...
     result1.t, result1.xref(1,:), 'k--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_1 [rad]');
title('Joint 1 Position'); legend(names{:}, 'Reference');

% Joint 2
subplot(2,2,2);
plot(result1.t, result1.x(3,:), 'b', ...
     result2.t, result2.x(3,:), 'r', ...
     result3.t, result3.x(3,:), 'g', ...
     result1.t, result1.xref(3,:), 'k--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_2 [rad]');
title('Joint 2 Position'); legend(names{:}, 'Reference');

% Control effort
subplot(2,2,3);
plot(result1.t, result1.u(1,:), 'b', ...
     result2.t, result2.u(1,:), 'r', ...
     result3.t, result3.u(1,:), 'g', 'LineWidth', 0.8);
xlabel('Time [s]'); ylabel('\tau_1 [N·m]');
title('Control Torque (Joint 1)'); legend(names{:});

% Tracking error
subplot(2,2,4);
plot(result1.t, result1.e(1,:), 'b', ...
     result2.t, result2.e(1,:), 'r', ...
     result3.t, result3.e(1,:), 'g', 'LineWidth', 0.8);
xlabel('Time [s]'); ylabel('e_1 [rad]');
title('Tracking Error (Joint 1)'); legend(names{:});

sgtitle('2-DOF Robot Arm: SMC Benchmark Comparison');
