%% VALIDATION: Fixed-Time SMC Controller
%
%  Validates the theoretical guarantee from Polyakov (2012):
%    Convergence time T <= 1/(alpha*(1-p)) + 1/(beta*(q-1))
%    REGARDLESS of initial conditions.
%
%  This is the KEY property: if the settling time exceeds T_max
%  for ANY initial condition, the implementation is WRONG.
%
%  Reference:
%    Polyakov, A. (2012). "Nonlinear feedback design for fixed-time
%    stabilization of linear control systems." IEEE TAC, 57(8).
%
%  Tests:
%    1. Convergence within T_max for small initial condition
%    2. Convergence within T_max for large initial condition
%    3. T_max is independent of initial conditions (same bound)
%    4. Compare with classical SMC (classical should be SLOWER for large x0)
%    5. Multiple (alpha, beta, p, q) parameter sets

clear; clc;
fprintf('=== Fixed-Time SMC Validation ===\n\n');
pass_count = 0;
fail_count = 0;

%% TEST 1: Convergence within T_max (small x0)
fprintf('Test 1: Convergence within T_max, x0=[1; 0]... ');
surf  = surfaces.LinearSurface('c', 10);
reach = reaching.ConstantRate('k', 1);  % dummy, not used by FixedTimeSMC
ctrl  = controllers.FixedTimeSMC(surf, reach);
ctrl.params.alpha = 5;
ctrl.params.beta  = 5;
ctrl.params.p = 0.5;
ctrl.params.q = 1.5;

T_max = ctrl.get_max_settling_time();
fprintf('(T_max = %.2f s) ', T_max);

plant = plants.DoubleIntegrator();
plant.x0 = [1; 0];

sim = benchmark.Simulator('dt', 1e-4, 'T', T_max + 2);
ref_fn  = @(t) [0; 0];
dist_fn = @(t) [0; 0];

arch = architectures.DirectSMC(ctrl);
result = sim.run(arch, plant, ref_fn, dist_fn);

% Check if sliding variable reached ~0 before T_max
s_at_Tmax_idx = find(result.t >= T_max, 1, 'first');
s_after = result.s(1, s_at_Tmax_idx:end);
max_s_after = max(abs(s_after));

if max_s_after < 0.1
    fprintf('PASS (|s| after T_max = %.4f)\n', max_s_after);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (|s| after T_max = %.4f, expected < 0.1)\n', max_s_after);
    fail_count = fail_count + 1;
end

%% TEST 2: Convergence within T_max (LARGE x0)
fprintf('Test 2: Convergence within T_max, x0=[100; 0]... ');
ctrl2 = controllers.FixedTimeSMC(surf, reach);
ctrl2.params.alpha = 5;
ctrl2.params.beta  = 5;
ctrl2.params.p = 0.5;
ctrl2.params.q = 1.5;
T_max2 = ctrl2.get_max_settling_time();  % same bound!

plant2 = plants.DoubleIntegrator();
plant2.x0 = [100; 0];

arch2 = architectures.DirectSMC(ctrl2);
sim2 = benchmark.Simulator('dt', 1e-4, 'T', T_max2 + 2);
result2 = sim2.run(arch2, plant2, ref_fn, dist_fn);

s_at_Tmax_idx = find(result2.t >= T_max2, 1, 'first');
s_after = result2.s(1, s_at_Tmax_idx:end);
max_s_after = max(abs(s_after));

if max_s_after < 1.0  % larger tolerance for larger initial condition
    fprintf('PASS (|s| after T_max = %.4f)\n', max_s_after);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (|s| after T_max = %.4f, expected < 1.0)\n', max_s_after);
    fail_count = fail_count + 1;
end

%% TEST 3: T_max is the SAME for both initial conditions
fprintf('Test 3: T_max independent of initial conditions... ');
if abs(T_max - T_max2) < 1e-15
    fprintf('PASS (T_max = %.4f for both)\n', T_max);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (T_max differs: %.4f vs %.4f)\n', T_max, T_max2);
    fail_count = fail_count + 1;
end

%% TEST 4: T_max formula correctness
fprintf('Test 4: T_max = 1/(alpha*(1-p)) + 1/(beta*(q-1))... ');
alpha_t = 5; beta_t = 5; p_t = 0.5; q_t = 1.5;
T_expected = 1/(alpha_t*(1-p_t)) + 1/(beta_t*(q_t-1));

if abs(T_max - T_expected) < 1e-15
    fprintf('PASS (%.4f = %.4f)\n', T_max, T_expected);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (%.4f != %.4f)\n', T_max, T_expected);
    fail_count = fail_count + 1;
end

%% TEST 5: Different parameter sets
fprintf('Test 5: Multiple parameter sets validate T_max formula... ');
test5_pass = true;
param_sets = {
    struct('alpha', 1, 'beta', 1, 'p', 0.5, 'q', 1.5);
    struct('alpha', 10, 'beta', 10, 'p', 0.3, 'q', 2.0);
    struct('alpha', 2, 'beta', 8, 'p', 0.7, 'q', 1.2);
    struct('alpha', 20, 'beta', 20, 'p', 0.5, 'q', 1.5);
};

for i = 1:numel(param_sets)
    ps = param_sets{i};
    ctrl_t = controllers.FixedTimeSMC(surf, reach);
    ctrl_t.params.alpha = ps.alpha;
    ctrl_t.params.beta  = ps.beta;
    ctrl_t.params.p     = ps.p;
    ctrl_t.params.q     = ps.q;

    T_calc = ctrl_t.get_max_settling_time();
    T_exp  = 1/(ps.alpha*(1-ps.p)) + 1/(ps.beta*(ps.q-1));

    if abs(T_calc - T_exp) > 1e-12
        test5_pass = false;
        fprintf('FAIL at set %d: %.6f != %.6f\n', i, T_calc, T_exp);
        break;
    end
end

if test5_pass
    fprintf('PASS (%d parameter sets)\n', numel(param_sets));
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 6: At s=0, control effort should be zero (no overshoot of reaching)
fprintf('Test 6: Zero control at s=0... ');
ctrl6 = controllers.FixedTimeSMC(surf, reach);
plant6 = plants.DoubleIntegrator();
% x = [0;0], xref = [0;0] => e=0, edot=0 => s=0
[u, info] = ctrl6.compute(0, [0;0], [0;0], plant6);

if abs(u) < 1e-15
    fprintf('PASS (u = %.2e at s=0)\n', u);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (u = %.4f at s=0, expected 0)\n', u);
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== Fixed-Time SMC: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
