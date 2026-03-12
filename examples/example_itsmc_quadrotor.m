%% OpenSMC ITSMC-RBF-ELM: Quadrotor Trajectory Tracking
%
%  Demonstrates the ITSMC controller with RBF-ELM disturbance estimation
%  on a 6-DOF quadrotor, using the CascadedSMC architecture:
%    - Outer loop: 3x ITSMC (one per translational axis) + RBF-ELM
%    - Control allocation: desired acceleration → thrust + angles
%    - Inner loop: PD attitude control
%
%  Scenario: Circular trajectory with sinusoidal altitude variation,
%  subject to harmonic external disturbances.
%
%  Run this script from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% 1. Plant
quad = plants.Quadrotor6DOF();
quad.x0 = zeros(12, 1);

%% 2. Simulation parameters
dt = 1e-3;
T  = 10;
t  = 0:dt:T;
N  = length(t);

%% 3. Reference Trajectory (circular with altitude variation)
ref_fn = utils.references.circular_3d(2.0, 0.5, 1.0, 0.3, 0.2);

% Pre-compute reference arrays for plotting
ref_x = zeros(1, N); ref_y = zeros(1, N); ref_z = zeros(1, N);
for j = 1:N
    xr = ref_fn(t(j));
    ref_x(j) = xr(1); ref_y(j) = xr(2); ref_z(j) = xr(3);
end
ref_psi = zeros(size(t));

%% 4. External Disturbance
rng(42);
d_ext_x = 0.2 * sin(2*t) + 0.05 * randn(1, N);
d_ext_y = 0.15 * cos(3*t) + 0.05 * randn(1, N);
d_ext_z = 0.1 * sin(t) + 0.03 * randn(1, N);

%% 5. Create RBF-ELM Estimators
est_x = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);
est_y = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);
est_z = estimators.RBF_ELM('n_hidden', 50, 'x_min', [-5 -8], 'x_max', [5 8]);

%% 6. Create ITSMC Controllers (per axis)
% Note: circular trajectory has centripetal acceleration ω²R ≈ 3.16 m/s²,
% so K must exceed this for the reaching law to maintain sliding.
dummy_surf  = surfaces.IntegralTerminalSurface('c1', 5, 'c2', 2, 'p', 5, 'q', 7);
dummy_reach = reaching.Saturation('k', 5, 'phi', 0.1);

ctrl_x = controllers.ITSMC(dummy_surf, dummy_reach, est_x);
ctrl_x.params.c1 = 5;  ctrl_x.params.c2 = 2;
ctrl_x.params.K = 5;   ctrl_x.params.lambda_s = 3;
ctrl_x.params.u_max = 20;
ctrl_x.params.dt = dt;

ctrl_y = controllers.ITSMC(dummy_surf, dummy_reach, est_y);
ctrl_y.params.c1 = 5;  ctrl_y.params.c2 = 2;
ctrl_y.params.K = 5;   ctrl_y.params.lambda_s = 3;
ctrl_y.params.u_max = 20;
ctrl_y.params.dt = dt;

ctrl_z = controllers.ITSMC(dummy_surf, dummy_reach, est_z);
ctrl_z.params.c1 = 5;  ctrl_z.params.c2 = 2;
ctrl_z.params.K = 3;   ctrl_z.params.lambda_s = 3;
ctrl_z.params.u_max = 20;
ctrl_z.params.dt = dt;

%% 7. Create Cascaded Architecture
arch = architectures.CascadedSMC(ctrl_x, ctrl_y, ctrl_z);

%% 8. Simulation Loop
X = zeros(N, 12);
X(1,:) = quad.x0';
U_log = zeros(N, 4);
S_log = zeros(N, 3);
D_log = zeros(N, 3);

fprintf('Running ITSMC-RBF-ELM quadrotor simulation (%d steps)...\n', N);

for k = 1:N-1
    x_k = X(k,:)';

    % Build full reference (12-state)
    xref = ref_fn(t(k));

    % Compute control
    [u_k, info_k] = arch.compute(t(k), x_k, xref, quad);

    % Disturbance (applied to translational accelerations: indices 7-9)
    d_k = zeros(12, 1);
    d_k(7) = d_ext_x(k) / quad.params.m;
    d_k(8) = d_ext_y(k) / quad.params.m;
    d_k(9) = d_ext_z(k) / quad.params.m;

    % RK4 integration
    f1 = quad.dynamics(t(k),         x_k,           u_k, d_k);
    f2 = quad.dynamics(t(k)+dt/2,    x_k+dt/2*f1,   u_k, d_k);
    f3 = quad.dynamics(t(k)+dt/2,    x_k+dt/2*f2,   u_k, d_k);
    f4 = quad.dynamics(t(k)+dt,      x_k+dt*f3,     u_k, d_k);
    X(k+1,:) = (x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4))';

    % Log
    U_log(k,:) = u_k';
    S_log(k,:) = info_k.s';
    D_log(k,:) = [info_k.dhat_x, info_k.dhat_y, info_k.dhat_z];
end

fprintf('Simulation complete.\n');

%% 9. Plot Results
figure('Name', 'ITSMC-RBF-ELM Quadrotor', 'Position', [50 50 1400 800]);

% 3D trajectory
subplot(2,4,1);
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5); hold on;
plot3(X(:,1), X(:,2), X(:,3), 'b-', 'LineWidth', 1);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Trajectory'); legend('Ref', 'Actual'); grid on;

% X tracking
subplot(2,4,2);
plot(t, ref_x, 'r--', t, X(:,1), 'b-', 'LineWidth', 1);
xlabel('t (s)'); ylabel('X (m)'); title('X Tracking'); grid on;

% Y tracking
subplot(2,4,3);
plot(t, ref_y, 'r--', t, X(:,2), 'b-', 'LineWidth', 1);
xlabel('t (s)'); ylabel('Y (m)'); title('Y Tracking'); grid on;

% Z tracking
subplot(2,4,4);
plot(t, ref_z, 'r--', t, X(:,3), 'b-', 'LineWidth', 1);
xlabel('t (s)'); ylabel('Z (m)'); title('Z Tracking'); grid on;

% Tracking errors
subplot(2,4,5);
plot(t, ref_x'-X(:,1), t, ref_y'-X(:,2), t, ref_z'-X(:,3), 'LineWidth', 1);
xlabel('t (s)'); ylabel('Error (m)'); title('Errors');
legend('e_x', 'e_y', 'e_z'); grid on;

% Sliding surfaces
subplot(2,4,6);
plot(t, S_log, 'LineWidth', 1);
xlabel('t (s)'); ylabel('s'); title('Sliding Surfaces');
legend('s_x', 's_y', 's_z'); grid on;

% Disturbance estimates
subplot(2,4,7);
plot(t, D_log, 'LineWidth', 1);
xlabel('t (s)'); ylabel('d_{hat}'); title('RBF-ELM Estimates');
legend('d_x', 'd_y', 'd_z'); grid on;

% Thrust
subplot(2,4,8);
plot(t, U_log(:,1), 'LineWidth', 1); hold on;
yline(quad.params.m * quad.params.g, 'r--', 'Hover');
xlabel('t (s)'); ylabel('U1 (N)'); title('Thrust'); grid on;

sgtitle('ITSMC + RBF-ELM on 6-DOF Quadrotor (OpenSMC)', 'FontSize', 14);

%% 10. Performance Metrics
ss = round(3/dt);  % steady-state after 3s
ex = ref_x' - X(:,1); ey = ref_y' - X(:,2); ez = ref_z' - X(:,3);

fprintf('\n=== ITSMC-RBF-ELM Performance (t > 3s) ===\n');
fprintf('RMSE  X: %.4f | Y: %.4f | Z: %.4f m\n', ...
    sqrt(mean(ex(ss:end).^2)), sqrt(mean(ey(ss:end).^2)), sqrt(mean(ez(ss:end).^2)));
fprintf('MAE   X: %.4f | Y: %.4f | Z: %.4f m\n', ...
    mean(abs(ex(ss:end))), mean(abs(ey(ss:end))), mean(abs(ez(ss:end))));
fprintf('Total RMSE: %.4f m\n', sqrt(mean(ex(ss:end).^2 + ey(ss:end).^2 + ez(ss:end).^2)));
