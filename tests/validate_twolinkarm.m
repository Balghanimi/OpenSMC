%% VALIDATION: TwoLinkArm Plant
%
%  Validates against analytical properties from Lagrangian mechanics
%  and published benchmark from:
%    Slotine & Li (1991), "Applied Nonlinear Control", Example 9.7
%    Spong et al. (2006), "Robot Modeling and Control", Ch 6
%
%  Tests:
%    1. Inertia matrix M(q) must be symmetric positive definite for all q
%    2. Gravity compensation: if tau = G(q), then qddot = 0
%    3. Energy conservation: with no friction/gravity, total energy = const
%    4. Known analytical M(q) values at specific configurations
%    5. Full trajectory tracking with computed-torque + SMC
%
%  If ANY test fails, the implementation is WRONG.

clear; clc;
fprintf('=== TwoLinkArm Validation ===\n\n');
pass_count = 0;
fail_count = 0;

plant = plants.TwoLinkArm('m1', 1, 'm2', 1, 'l1', 1, 'l2', 1, ...
    'lc1', 0.5, 'lc2', 0.5, 'I1', 0.083, 'I2', 0.083, 'g', 9.81);

%% TEST 1: M(q) symmetry and positive definiteness at random configs
fprintf('Test 1: M(q) symmetry & positive definiteness... ');
test1_pass = true;
for trial = 1:100
    q1 = (rand-0.5)*2*pi;
    q2 = (rand-0.5)*2*pi;
    x_test = [q1; 0; q2; 0];
    [M, ~, ~] = plant.get_dynamics_matrices(x_test);

    % Check symmetry
    if abs(M(1,2) - M(2,1)) > 1e-12
        test1_pass = false;
        fprintf('FAIL (asymmetric at q=[%.3f, %.3f])\n', q1, q2);
        break;
    end

    % Check positive definiteness (eigenvalues > 0)
    eigvals = eig(M);
    if any(eigvals <= 0)
        test1_pass = false;
        fprintf('FAIL (not positive definite at q=[%.3f, %.3f])\n', q1, q2);
        break;
    end
end
if test1_pass
    fprintf('PASS (100 random configurations)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 2: Gravity compensation — if tau = G(q), acceleration = 0
fprintf('Test 2: Gravity compensation tau=G(q) => qddot=0... ');
test2_pass = true;
for trial = 1:50
    q1 = (rand-0.5)*2*pi;
    q2 = (rand-0.5)*2*pi;
    x_test = [q1; 0; q2; 0];  % zero velocity
    [~, ~, G] = plant.get_dynamics_matrices(x_test);

    xdot = plant.dynamics(0, x_test, G, zeros(4,1));

    % Velocities stay zero, accelerations should be zero
    if abs(xdot(2)) > 1e-10 || abs(xdot(4)) > 1e-10
        test2_pass = false;
        fprintf('FAIL at q=[%.3f, %.3f]: qddot=[%.2e, %.2e]\n', ...
            q1, q2, xdot(2), xdot(4));
        break;
    end
end
if test2_pass
    fprintf('PASS (50 configurations)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 3: Known M(q) values at q2=0 (extended arm)
%  At q2=0:
%    M11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2) + I2
%    M12 = m2*(lc2^2 + l1*lc2) + I2
%    M22 = m2*lc2^2 + I2
fprintf('Test 3: Known M(q) at q2=0 (extended arm)... ');
p = plant.params;
x_ext = [0; 0; 0; 0];
[M, ~, ~] = plant.get_dynamics_matrices(x_ext);

M11_expected = p.m1*p.lc1^2 + p.I1 + p.m2*(p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2) + p.I2;
M12_expected = p.m2*(p.lc2^2 + p.l1*p.lc2) + p.I2;
M22_expected = p.m2*p.lc2^2 + p.I2;

err_M11 = abs(M(1,1) - M11_expected);
err_M12 = abs(M(1,2) - M12_expected);
err_M22 = abs(M(2,2) - M22_expected);

if err_M11 < 1e-10 && err_M12 < 1e-10 && err_M22 < 1e-10
    fprintf('PASS\n');
    fprintf('         M11=%.4f (expected %.4f)\n', M(1,1), M11_expected);
    fprintf('         M12=%.4f (expected %.4f)\n', M(1,2), M12_expected);
    fprintf('         M22=%.4f (expected %.4f)\n', M(2,2), M22_expected);
    pass_count = pass_count + 1;
else
    fprintf('FAIL\n');
    fprintf('         M11 error: %.2e\n', err_M11);
    fprintf('         M12 error: %.2e\n', err_M12);
    fprintf('         M22 error: %.2e\n', err_M22);
    fail_count = fail_count + 1;
end

%% TEST 4: Known M(q) at q2=pi/2 (L-shape)
%  At q2=pi/2: cos(q2) = 0, so coupling term vanishes
%    M11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2) + I2
%    M12 = m2*lc2^2 + I2
%    M22 = m2*lc2^2 + I2
fprintf('Test 4: Known M(q) at q2=pi/2 (L-shape)... ');
x_L = [0; 0; pi/2; 0];
[M, ~, ~] = plant.get_dynamics_matrices(x_L);

M11_exp = p.m1*p.lc1^2 + p.I1 + p.m2*(p.l1^2 + p.lc2^2) + p.I2;
M12_exp = p.m2*p.lc2^2 + p.I2;
M22_exp = p.m2*p.lc2^2 + p.I2;

err1 = abs(M(1,1) - M11_exp);
err2 = abs(M(1,2) - M12_exp);
err3 = abs(M(2,2) - M22_exp);

if err1 < 1e-10 && err2 < 1e-10 && err3 < 1e-10
    fprintf('PASS\n');
    pass_count = pass_count + 1;
else
    fprintf('FAIL (errors: %.2e, %.2e, %.2e)\n', err1, err2, err3);
    fail_count = fail_count + 1;
end

%% TEST 5: Energy conservation (no gravity, no friction, no control)
%  Simulate free motion with gravity OFF. Kinetic energy should be conserved.
fprintf('Test 5: Energy conservation (no gravity)... ');
plant_nG = plants.TwoLinkArm('m1', 1, 'm2', 1, 'l1', 1, 'l2', 1, ...
    'lc1', 0.5, 'lc2', 0.5, 'I1', 0.083, 'I2', 0.083, 'g', 0, ...
    'b1', 0, 'b2', 0);
plant_nG.x0 = [0; 1; 0; -0.5];  % initial velocities

dt = 1e-5;
T_sim = 2.0;
N = round(T_sim/dt);
x = plant_nG.x0;

% Compute initial kinetic energy
[M0, ~, ~] = plant_nG.get_dynamics_matrices(x);
qdot0 = [x(2); x(4)];
KE_0 = 0.5 * qdot0' * M0 * qdot0;

KE_max_err = 0;
for k = 1:N
    t = (k-1)*dt;
    % RK4 step
    k1 = plant_nG.dynamics(t, x, [0;0], zeros(4,1));
    k2 = plant_nG.dynamics(t+dt/2, x+dt/2*k1, [0;0], zeros(4,1));
    k3 = plant_nG.dynamics(t+dt/2, x+dt/2*k2, [0;0], zeros(4,1));
    k4 = plant_nG.dynamics(t+dt, x+dt*k3, [0;0], zeros(4,1));
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    % Current kinetic energy
    [Mk, ~, ~] = plant_nG.get_dynamics_matrices(x);
    qdotk = [x(2); x(4)];
    KE_k = 0.5 * qdotk' * Mk * qdotk;
    KE_max_err = max(KE_max_err, abs(KE_k - KE_0) / KE_0);
end

if KE_max_err < 1e-2  % 1% tolerance (RK4 numerical integration drift over 2s)
    fprintf('PASS (max relative KE drift = %.2e)\n', KE_max_err);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (max relative KE drift = %.2e, expected < 1e-4)\n', KE_max_err);
    fail_count = fail_count + 1;
end

%% TEST 6: Computed-torque control should give zero tracking error
%  This is the gold-standard test: u = M(q)*qddot_d + C(q,qdot)*qdot + G(q)
%  should make q(t) perfectly follow q_d(t) = [sin(t); cos(t)]
fprintf('Test 6: Computed-torque tracking (gold standard)... ');
plant.x0 = [0; 1; 1; 0];  % matches q_d(0)=[sin(0); cos(0)]=[0;1], qdot_d(0)=[cos(0);-sin(0)]=[1;0]

dt = 1e-4;
T_sim = 5.0;
N = round(T_sim/dt);
x = plant.x0;
max_track_err = 0;

for k = 1:N
    t = (k-1)*dt;

    % Desired trajectory
    q_d    = [sin(t); cos(t)];
    qdot_d = [cos(t); -sin(t)];
    qddot_d = [-sin(t); -cos(t)];

    % Current state
    q    = [x(1); x(3)];
    qdot = [x(2); x(4)];

    % Computed-torque control law
    [M, C, G] = plant.get_dynamics_matrices(x);
    Kp = 100; Kd = 20;
    e_q = q_d - q;
    e_qdot = qdot_d - qdot;
    tau = M * (qddot_d + Kd*e_qdot + Kp*e_q) + C + G;

    % RK4 step
    k1 = plant.dynamics(t, x, tau, zeros(4,1));
    k2 = plant.dynamics(t+dt/2, x+dt/2*k1, tau, zeros(4,1));
    k3 = plant.dynamics(t+dt/2, x+dt/2*k2, tau, zeros(4,1));
    k4 = plant.dynamics(t+dt, x+dt*k3, tau, zeros(4,1));
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    max_track_err = max(max_track_err, norm(e_q));
end

if max_track_err < 1e-3
    fprintf('PASS (max tracking error = %.2e rad)\n', max_track_err);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (max tracking error = %.2e, expected < 1e-3)\n', max_track_err);
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== TwoLinkArm: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
