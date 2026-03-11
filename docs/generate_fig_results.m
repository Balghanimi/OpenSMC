%% Generate Results Figure for SoftwareX Paper
%  Runs the ITSMC-RBF-ELM quadrotor simulation and exports a 4-panel PDF.
%  Run from OpenSMC root: matlab -batch "addpath(pwd); run('docs/generate_fig_results.m')"

clear; clc; close all;

%% 1. Plant
quad = plants.Quadrotor6DOF();
quad.x0 = zeros(12, 1);

%% 2. Simulation parameters
dt = 1e-3;
T  = 10;
t  = 0:dt:T;
N  = length(t);

%% 3. Reference Trajectory
ramp = 1 ./ (1 + exp(-3*(t - 2)));
ref_x   = ramp .* 2 .* cos(0.5 * t);
ref_y   = ramp .* 2 .* sin(0.5 * t);
ref_z   = ramp .* (1 + 0.3 * sin(0.2 * t));
ref_psi = zeros(size(t));
ref_dx = [0, diff(ref_x)] / dt;
ref_dy = [0, diff(ref_y)] / dt;
ref_dz = [0, diff(ref_z)] / dt;

%% 4. External Disturbance
rng(42);
d_ext_x = 0.2 * sin(2*t) + 0.05 * randn(1, N);
d_ext_y = 0.15 * cos(3*t) + 0.05 * randn(1, N);
d_ext_z = 0.1 * sin(t) + 0.03 * randn(1, N);

%% 5. Controllers
dummy_surf  = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1, 'p', 5, 'q', 7);
dummy_reach = reaching.Saturation('k', 0.5, 'phi', 0.2);

est_x = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);
est_y = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);
est_z = estimators.RBF_ELM('n_hidden', 25, 'x_min', [-3 -5], 'x_max', [3 5]);

ctrl_x = controllers.ITSMC(dummy_surf, dummy_reach, est_x);
ctrl_x.params.c1 = 3; ctrl_x.params.c2 = 1;
ctrl_x.params.K = 0.5; ctrl_x.params.lambda_s = 2; ctrl_x.params.dt = dt;

ctrl_y = controllers.ITSMC(dummy_surf, dummy_reach, est_y);
ctrl_y.params.c1 = 3; ctrl_y.params.c2 = 1;
ctrl_y.params.K = 0.5; ctrl_y.params.lambda_s = 2; ctrl_y.params.dt = dt;

ctrl_z = controllers.ITSMC(dummy_surf, dummy_reach, est_z);
ctrl_z.params.c1 = 4; ctrl_z.params.c2 = 2;
ctrl_z.params.K = 1.0; ctrl_z.params.lambda_s = 2; ctrl_z.params.dt = dt;

arch = architectures.CascadedSMC(ctrl_x, ctrl_y, ctrl_z);

%% 6. Simulation
X = zeros(N, 12);
X(1,:) = quad.x0';
U_log = zeros(N, 4);
S_log = zeros(N, 3);
D_log = zeros(N, 3);

fprintf('Running simulation for figure export (%d steps)...\n', N);

for k = 1:N-1
    x_k = X(k,:)';
    xref = [ref_x(k); ref_y(k); ref_z(k); 0; 0; ref_psi(k); ...
            ref_dx(k); ref_dy(k); ref_dz(k); 0; 0; 0];
    [u_k, info_k] = arch.compute(t(k), x_k, xref, quad);

    d_k = zeros(12, 1);
    d_k(7) = d_ext_x(k) / quad.params.m;
    d_k(8) = d_ext_y(k) / quad.params.m;
    d_k(9) = d_ext_z(k) / quad.params.m;

    f1 = quad.dynamics(t(k),       x_k,          u_k, d_k);
    f2 = quad.dynamics(t(k)+dt/2,  x_k+dt/2*f1,  u_k, d_k);
    f3 = quad.dynamics(t(k)+dt/2,  x_k+dt/2*f2,  u_k, d_k);
    f4 = quad.dynamics(t(k)+dt,    x_k+dt*f3,     u_k, d_k);
    X(k+1,:) = (x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4))';

    U_log(k,:) = u_k';
    S_log(k,:) = info_k.s';
    D_log(k,:) = [info_k.dhat_x, info_k.dhat_y, info_k.dhat_z];
end

fprintf('Simulation complete. Generating figure...\n');

%% 7. Create Publication-Quality Figure
fig = figure('Units', 'centimeters', 'Position', [2 2 18 16], ...
    'PaperUnits', 'centimeters', 'PaperSize', [18 16], ...
    'PaperPosition', [0 0 18 16], 'Color', 'w');

% Color palette
c_ref  = [0.85 0.15 0.15];  % red
c_act  = [0.15 0.25 0.85];  % blue
c_x    = [0.85 0.15 0.15];
c_y    = [0.15 0.65 0.30];
c_z    = [0.15 0.25 0.85];
lw = 1.0;

% (a) 3D Trajectory
ax1 = subplot(2, 2, 1);
plot3(ref_x, ref_y, ref_z, '--', 'Color', c_ref, 'LineWidth', 1.4); hold on;
plot3(X(:,1), X(:,2), X(:,3), '-', 'Color', c_act, 'LineWidth', lw);
plot3(X(1,1), X(1,2), X(1,3), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
xlabel('$x$ (m)', 'Interpreter', 'latex');
ylabel('$y$ (m)', 'Interpreter', 'latex');
zlabel('$z$ (m)', 'Interpreter', 'latex');
legend('Reference', 'ITSMC-ELM', 'Start', 'Location', 'northwest', ...
    'FontSize', 7, 'Box', 'off');
title('(a) 3D Trajectory', 'FontSize', 9);
grid on; view(35, 25);
set(ax1, 'FontSize', 8);

% (b) Tracking Errors
ax2 = subplot(2, 2, 2);
ex = ref_x' - X(:,1); ey = ref_y' - X(:,2); ez = ref_z' - X(:,3);
plot(t, ex, '-', 'Color', c_x, 'LineWidth', lw); hold on;
plot(t, ey, '-', 'Color', c_y, 'LineWidth', lw);
plot(t, ez, '-', 'Color', c_z, 'LineWidth', lw);
xlabel('Time (s)', 'FontSize', 8);
ylabel('Error (m)', 'FontSize', 8);
legend('$e_x$', '$e_y$', '$e_z$', 'Interpreter', 'latex', ...
    'FontSize', 7, 'Location', 'northeast', 'Box', 'off');
title('(b) Tracking Errors', 'FontSize', 9);
grid on; xlim([0 T]);
set(ax2, 'FontSize', 8);

% (c) Sliding Variables
ax3 = subplot(2, 2, 3);
plot(t, S_log(:,1), '-', 'Color', c_x, 'LineWidth', lw); hold on;
plot(t, S_log(:,2), '-', 'Color', c_y, 'LineWidth', lw);
plot(t, S_log(:,3), '-', 'Color', c_z, 'LineWidth', lw);
yline(0, 'k:', 'LineWidth', 0.5);
xlabel('Time (s)', 'FontSize', 8);
ylabel('$s$', 'Interpreter', 'latex', 'FontSize', 8);
legend('$s_x$', '$s_y$', '$s_z$', 'Interpreter', 'latex', ...
    'FontSize', 7, 'Location', 'northeast', 'Box', 'off');
title('(c) Sliding Variables', 'FontSize', 9);
grid on; xlim([0 T]);
set(ax3, 'FontSize', 8);

% (d) Disturbance Estimates vs Actual
ax4 = subplot(2, 2, 4);
plot(t, d_ext_x, '-', 'Color', [c_x 0.3], 'LineWidth', 0.5); hold on;
plot(t, D_log(:,1), '-', 'Color', c_x, 'LineWidth', lw);
plot(t, d_ext_y, '-', 'Color', [c_y 0.3], 'LineWidth', 0.5);
plot(t, D_log(:,2), '-', 'Color', c_y, 'LineWidth', lw);
plot(t, d_ext_z, '-', 'Color', [c_z 0.3], 'LineWidth', 0.5);
plot(t, D_log(:,3), '-', 'Color', c_z, 'LineWidth', lw);
xlabel('Time (s)', 'FontSize', 8);
ylabel('Disturbance', 'FontSize', 8);
legend('$d_x$ actual', '$\hat{d}_x$ ELM', ...
       '$d_y$ actual', '$\hat{d}_y$ ELM', ...
       '$d_z$ actual', '$\hat{d}_z$ ELM', ...
    'Interpreter', 'latex', 'FontSize', 6, 'Location', 'northeast', ...
    'NumColumns', 2, 'Box', 'off');
title('(d) RBF-ELM Disturbance Estimation', 'FontSize', 9);
grid on; xlim([0 T]);
set(ax4, 'FontSize', 8);

%% 8. Export
outpath = fullfile(fileparts(mfilename('fullpath')), 'fig_results.pdf');
exportgraphics(fig, outpath, 'ContentType', 'vector', 'Resolution', 300);
fprintf('Figure saved to: %s\n', outpath);

% Also save as EPS for journals that prefer it
outpath_eps = fullfile(fileparts(mfilename('fullpath')), 'fig_results.eps');
exportgraphics(fig, outpath_eps, 'ContentType', 'vector');
fprintf('EPS saved to: %s\n', outpath_eps);

close(fig);
fprintf('Done.\n');
