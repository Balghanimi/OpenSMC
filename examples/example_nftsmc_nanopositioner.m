%% OpenSMC: NFTSMC Vibration Suppression on Dual-Stage Nanopositioner
%
%  Demonstrates nonsingular fast terminal sliding mode control on a
%  piezoelectric nanopositioner with two resonant modes (780 Hz, 2340 Hz).
%
%  Three test scenarios:
%    1. Step response (1 um) — settling time and overshoot
%    2. Vibration rejection (780 Hz sinusoidal disturbance)
%    3. Triangular tracking + vibration — RMS error comparison
%
%  Compares NFTSMC against ClassicalSMC baseline.
%
%  Run from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% Plant
nano = plants.DualStageNanopositioner();
fprintf('Plant: %s\n', nano.name);
fprintf('  Input gain g = %.4f\n', nano.get_input_gain());
fprintf('  Mode 1: fn = %d Hz, z = %.3f, K = %.3e m/V\n', ...
    nano.params.fn1, nano.params.z1, nano.params.Kp1);
fprintf('  Mode 2: fn = %d Hz, z = %.3f, K = %.3e m/V\n', ...
    nano.params.fn2, nano.params.z2, nano.params.Kp2);

%% Simulation parameters
dt = 1e-5;   % 100 kHz (matches nanopositioner bandwidth)
wn1 = nano.params.wn1;

%% Controller 1: NFTSMC (full, with feedforward + integral)
surf1  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
reach1 = reaching.PowerRate('k', 80, 'alpha', 5/7);
ctrl1  = controllers.NFTSMC(surf1, reach1);

% Configure for nanopositioner
g = nano.get_input_gain();
p = nano.params;
ff_corr = abs(g) / (p.Kp1 + p.Kp2) - p.wn1^2;  % feedforward correction

ctrl1.params.input_gain     = g;
ctrl1.params.use_feedforward = true;
ctrl1.params.wn             = p.wn1;
ctrl1.params.zeta           = p.z1;
ctrl1.params.ff_corr        = ff_corr;
ctrl1.params.kI             = 5e8;
ctrl1.params.dt             = dt;
ctrl1.params.u_max          = 20;

arch1 = architectures.DirectSMC(ctrl1);

%% Controller 2: ClassicalSMC baseline (Linear surface + Super-twisting)
surf2  = surfaces.LinearSurface('c', 500);
reach2 = reaching.SuperTwisting('k1', 300, 'k2', 150);
ctrl2  = controllers.ClassicalSMC(surf2, reach2);
arch2  = architectures.DirectSMC(ctrl2);

%% ========== Scenario 1: Step Response ==========
fprintf('\n--- Scenario 1: Step Response (1 um) ---\n');
T_step = 0.05;  % 50 ms
ref_step = @(t) [1e-6 * (t >= 1e-3); 0; 0; 0];
dist_none = @(t) zeros(4, 1);

sim1 = benchmark.Simulator('dt', dt, 'T', T_step);
r1_nf = sim1.run(arch1, nano, ref_step, dist_none);
arch1.reset(); arch2.reset();
r1_cl = sim1.run(arch2, nano, ref_step, dist_none);

m1_nf = benchmark.Metrics.compute_all(r1_nf);
m1_cl = benchmark.Metrics.compute_all(r1_cl);

fprintf('  NFTSMC:      RMSE = %.3e m, Settling = %.1f ms\n', ...
    m1_nf.rmse, m1_nf.settling_time * 1e3);
fprintf('  ClassicalSMC: RMSE = %.3e m, Settling = %.1f ms\n', ...
    m1_cl.rmse, m1_cl.settling_time * 1e3);

%% ========== Scenario 2: Vibration Rejection ==========
fprintf('\n--- Scenario 2: Vibration Rejection (780 Hz) ---\n');
T_vib = 0.04;
ref_zero = @(t) zeros(4, 1);
dist_vib = @(t) [0.1e-6*sin(wn1*t); 0; 0; 0] * (t >= 5e-3);

arch1.reset(); arch2.reset();
sim2 = benchmark.Simulator('dt', dt, 'T', T_vib);
r2_nf = sim2.run(arch1, nano, ref_zero, dist_vib);
arch1.reset(); arch2.reset();
r2_cl = sim2.run(arch2, nano, ref_zero, dist_vib);

% Steady-state vibration amplitude
i_ss = round(0.8 * size(r2_nf.x, 2)):size(r2_nf.x, 2);
ss_nf = max(abs(r2_nf.x(1,i_ss) + r2_nf.x(3,i_ss)));
ss_cl = max(abs(r2_cl.x(1,i_ss) + r2_cl.x(3,i_ss)));

fprintf('  NFTSMC:      SS vibration = %.4f um\n', ss_nf * 1e6);
fprintf('  ClassicalSMC: SS vibration = %.4f um\n', ss_cl * 1e6);

%% ========== Scenario 3: Tracking + Vibration ==========
fprintf('\n--- Scenario 3: Triangular Tracking + Vibration ---\n');
T_track = 0.06;
tri_fn = @(t) tri_wave(t, 50, 5e-6);
ref_track = @(t) [tri_fn(t); 0; 0; 0];
dist_track = @(t) [0.05e-6*sin(wn1*t); 0; 0; 0];

arch1.reset(); arch2.reset();
sim3 = benchmark.Simulator('dt', dt, 'T', T_track);
r3_nf = sim3.run(arch1, nano, ref_track, dist_track);
arch1.reset(); arch2.reset();
r3_cl = sim3.run(arch2, nano, ref_track, dist_track);

m3_nf = benchmark.Metrics.compute_all(r3_nf);
m3_cl = benchmark.Metrics.compute_all(r3_cl);

fprintf('  NFTSMC:      RMSE = %.1f nm\n', m3_nf.rmse * 1e9);
fprintf('  ClassicalSMC: RMSE = %.1f nm\n', m3_cl.rmse * 1e9);

%% ========== Plots ==========
figure('Position', [100 100 1000 700]);

% Step response
subplot(2,2,1);
plot(r1_nf.t*1e3, (r1_nf.x(1,:)+r1_nf.x(3,:))*1e6, 'b-', 'LineWidth', 1.5); hold on;
plot(r1_cl.t*1e3, (r1_cl.x(1,:)+r1_cl.x(3,:))*1e6, 'r--', 'LineWidth', 1.2);
yline(1, 'k:', 'LineWidth', 0.8);
xlabel('Time (ms)'); ylabel('Displacement (um)');
title('Step Response'); legend('NFTSMC', 'ClassicalSMC', 'Location', 'southeast');
grid on;

% Vibration rejection
subplot(2,2,2);
plot(r2_nf.t*1e3, (r2_nf.x(1,:)+r2_nf.x(3,:))*1e6, 'b-', 'LineWidth', 1.2); hold on;
plot(r2_cl.t*1e3, (r2_cl.x(1,:)+r2_cl.x(3,:))*1e6, 'r--', 'LineWidth', 1.2);
xlabel('Time (ms)'); ylabel('Displacement (um)');
title('Vibration Rejection (780 Hz)'); legend('NFTSMC', 'ClassicalSMC');
grid on;

% Tracking error
subplot(2,2,3);
e_nf = r3_nf.xref(1,:) - (r3_nf.x(1,:)+r3_nf.x(3,:));
e_cl = r3_cl.xref(1,:) - (r3_cl.x(1,:)+r3_cl.x(3,:));
plot(r3_nf.t*1e3, e_nf*1e9, 'b-', 'LineWidth', 0.8); hold on;
plot(r3_cl.t*1e3, e_cl*1e9, 'r--', 'LineWidth', 0.8);
xlabel('Time (ms)'); ylabel('Error (nm)');
title('Tracking Error'); legend('NFTSMC', 'ClassicalSMC');
grid on;

% Control signals
subplot(2,2,4);
plot(r1_nf.t*1e3, r1_nf.u(1,:), 'b-', 'LineWidth', 0.8); hold on;
plot(r1_cl.t*1e3, r1_cl.u(1,:), 'r--', 'LineWidth', 0.8);
xlabel('Time (ms)'); ylabel('Voltage (V)');
title('Control Signal (Step)'); legend('NFTSMC', 'ClassicalSMC');
grid on;

sgtitle('NFTSMC vs ClassicalSMC on DualStageNanopositioner');

fprintf('\nDone.\n');

%% --- Helper ---
function y = tri_wave(t, f, a)
    p = mod(t * f, 1);
    if p < 0.5
        y = a * (4*p - 1);
    else
        y = a * (3 - 4*p);
    end
end
