%% OpenSMC Crane HSMC: Compare Hierarchical SMC Methods on Overhead Crane
%
%  Demonstrates the three hierarchical SMC variants from Qian & Yi (2015)
%  applied to the single-pendulum overhead crane:
%    1. Aggregated HSMC  (two-layer, physics-based decomposition)
%    2. Incremental HSMC (three-layer, incremental state addition)
%    3. Combining HSMC   (intermediate variable + derivative)
%
%  The crane is underactuated: 1 input (trolley force), 2 DOF (trolley + pendulum).
%  Task: transport trolley from x=0 to x=2m while suppressing payload swing.
%
%  Run this script from the OpenSMC root directory.

clear; clc; close all;
addpath('..');

%% Plant: Single-Pendulum Crane (Qian Table 4.1 parameters)
crane = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
crane.x0 = [0; 0; 0; 0];   % start at rest

%% Reference: move trolley to x=2m, zero swing
xd = [2; 0; 0; 0];  % desired state
ref_fn = @(t, n) xd;

%% No external disturbance
dist_fn = @(t, n) zeros(n, 1);

%% Simulation parameters
dt = 1e-4;
T  = 8;        % 8 seconds of simulation
N  = round(T/dt);

%% Dummy surface/reaching (HSMC controllers compute internally)
dummy_surf = surfaces.LinearSurface('c', 1);
dummy_reach = reaching.ConstantRate('k', 1);

%% Controller 1: Aggregated HSMC (Sec 4.2)
ctrl1 = controllers.AggregatedHSMC(dummy_surf, dummy_reach);
ctrl1.params.c1    = 0.7;
ctrl1.params.c2    = 8.2;
ctrl1.params.alpha = -2.3;
ctrl1.params.kappa = 3;
ctrl1.params.eta   = 0.1;

%% Controller 2: Incremental HSMC (Sec 4.3)
ctrl2 = controllers.IncrementalHSMC(dummy_surf, dummy_reach);
ctrl2.params.c1    = 0.85;
ctrl2.params.c2    = 3.6;
ctrl2.params.c3    = 0.4;
ctrl2.params.kappa = 3;
ctrl2.params.eta   = 0.1;

%% Controller 3: Combining HSMC (Sec 4.4)
ctrl3 = controllers.CombiningHSMC(dummy_surf, dummy_reach);
ctrl3.params.c     = 0.242;
ctrl3.params.alpha = 0.487;
ctrl3.params.kappa = 4;
ctrl3.params.eta   = 0.1;

%% Run simulations
ctrls  = {ctrl1, ctrl2, ctrl3};
labels = {'Aggregated HSMC', 'Incremental HSMC', 'Combining HSMC'};
colors = {'b', 'r', 'g'};

results = cell(1, 3);
for k = 1:3
    x = crane.x0;
    ctrls{k}.reset();

    % Storage
    t_log = zeros(1, N);
    x_log = zeros(4, N);
    u_log = zeros(1, N);

    for i = 1:N
        t = (i-1)*dt;
        xref = ref_fn(t, 4);
        d = dist_fn(t, 4);

        [u, info] = ctrls{k}.compute(t, x, xref, crane);

        % Saturate control
        u = max(-200, min(200, u));

        % Log
        t_log(i) = t;
        x_log(:,i) = x;
        u_log(i) = u;

        % RK4 integration
        k1 = crane.dynamics(t, x, u, d);
        k2 = crane.dynamics(t+dt/2, x+dt/2*k1, u, d);
        k3 = crane.dynamics(t+dt/2, x+dt/2*k2, u, d);
        k4 = crane.dynamics(t+dt, x+dt*k3, u, d);
        x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    end

    results{k}.t = t_log;
    results{k}.x = x_log;
    results{k}.u = u_log;
end

%% Plot results
figure('Name', 'Crane HSMC Comparison', 'Position', [100 100 900 700]);

% Trolley position
subplot(3,2,1);
for k = 1:3
    plot(results{k}.t, results{k}.x(1,:), colors{k}, 'LineWidth', 1.2); hold on;
end
yline(xd(1), 'k--'); ylabel('x (m)'); title('Trolley Position');
legend(labels{:}, 'Location', 'best'); grid on;

% Trolley velocity
subplot(3,2,2);
for k = 1:3
    plot(results{k}.t, results{k}.x(2,:), colors{k}, 'LineWidth', 1.2); hold on;
end
ylabel('v (m/s)'); title('Trolley Velocity'); grid on;

% Payload angle
subplot(3,2,3);
for k = 1:3
    plot(results{k}.t, rad2deg(results{k}.x(3,:)), colors{k}, 'LineWidth', 1.2); hold on;
end
ylabel('\theta (deg)'); title('Payload Angle'); grid on;

% Payload angular velocity
subplot(3,2,4);
for k = 1:3
    plot(results{k}.t, rad2deg(results{k}.x(4,:)), colors{k}, 'LineWidth', 1.2); hold on;
end
ylabel('\omega (deg/s)'); title('Payload Angular Velocity'); grid on;

% Control input
subplot(3,2,5);
for k = 1:3
    plot(results{k}.t, results{k}.u, colors{k}, 'LineWidth', 1.2); hold on;
end
xlabel('Time (s)'); ylabel('u (N)'); title('Control Force'); grid on;

% Metrics
subplot(3,2,6); axis off;
text(0.1, 0.9, 'Performance Metrics:', 'FontWeight', 'bold', 'FontSize', 11);
for k = 1:3
    % Settling time (2% criterion)
    final_pos = results{k}.x(1,end);
    max_swing = max(abs(results{k}.x(3,:)));
    ise = trapz(results{k}.t, (results{k}.x(1,:) - xd(1)).^2);

    text(0.1, 0.7-0.25*(k-1), sprintf('%s:', labels{k}), ...
        'Color', colors{k}, 'FontWeight', 'bold');
    text(0.1, 0.6-0.25*(k-1), sprintf('  Final pos: %.3f m, Max swing: %.2f deg, ISE: %.3f', ...
        final_pos, rad2deg(max_swing), ise));
end

sgtitle('Hierarchical SMC Comparison on Overhead Crane (Qian & Yi 2015)');
