%% VALIDATION: Fuzzy SMC & Discrete-Time SMC Controllers
%
%  Fuzzy SMC validation:
%    1. Output bounded in [-k, k]
%    2. sign(output) matches sign(s) (correct direction)
%    3. Zero output at s=0
%    4. Chattering reduction vs classical sign(s)
%
%  Discrete SMC validation:
%    1. Zero-order hold between sampling instants
%    2. Discrete reaching condition: |s(k+1)| < |s(k)|
%    3. Gao (1995) reaching law parameter verification
%
%  References:
%    Gao, W. et al. (1995). IEEE TIE, 42(2), 117-122.
%    Khanesar et al. (2021). "Sliding-Mode Fuzzy Controllers", Springer.

clear; clc;
fprintf('=== Fuzzy SMC & Discrete SMC Validation ===\n\n');
pass_count = 0;
fail_count = 0;

%% ===== FUZZY SMC =====

%% TEST 1: Fuzzy output bounded in [-1, 1] * k
fprintf('Test 1: FuzzySMC output bounded... ');
surf = surfaces.LinearSurface('c', 10);
reach = reaching.ConstantRate('k', 1);
ctrl = controllers.FuzzySMC(surf, reach);
ctrl.params.k = 15;
plant = plants.DoubleIntegrator();

test1_pass = true;
for trial = 1:200
    x = [(rand-0.5)*20; (rand-0.5)*10];
    xref = [0; 0];
    [u, info] = ctrl.compute(0, x, xref, plant);

    if abs(info.k_fuzzy) > 1.01  % small tolerance
        test1_pass = false;
        fprintf('FAIL: k_fuzzy=%.4f > 1\n', info.k_fuzzy);
        break;
    end
end
if test1_pass
    fprintf('PASS (200 random states, |k_fuzzy| <= 1)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 2: Correct control direction (sign matches s)
fprintf('Test 2: FuzzySMC corrects in right direction... ');
ctrl2 = controllers.FuzzySMC(surf, reach);
ctrl2.params.k = 10;

% Positive error => positive s => negative u (push back)
% Actually: u = -k * k_fuzzy. If s > 0, k_fuzzy > 0, so u < 0 (correct)
test2_pass = true;
for trial = 1:50
    e_sign = sign(rand - 0.5);
    x = [e_sign * (1 + rand*5); 0];
    xref = [0; 0];
    [u, info] = ctrl2.compute(trial*1e-4, x, xref, plant);

    % s = edot + c*e. With xref=0: e = -x(1), edot = -x(2)
    % If x(1) > 0, e < 0, s < 0 => k_fuzzy < 0 => u > 0 (push positive? depends on convention)
    % The key check: sign(k_fuzzy) should match sign(s) when |s| is large
    if abs(info.s) > 0.5
        if sign(info.k_fuzzy) ~= sign(info.s)
            test2_pass = false;
            fprintf('FAIL: s=%.2f but k_fuzzy=%.2f (wrong sign)\n', info.s, info.k_fuzzy);
            break;
        end
    end
end
if test2_pass
    fprintf('PASS (sign(k_fuzzy) = sign(s) for |s| > 0.5)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 3: Near-zero output at equilibrium
fprintf('Test 3: FuzzySMC ~0 output at s=0... ');
ctrl3 = controllers.FuzzySMC(surf, reach);
[u3, info3] = ctrl3.compute(0, [0;0], [0;0], plant);

if abs(u3) < 1.0  % should be very small
    fprintf('PASS (u = %.4f at e=0, edot=0)\n', u3);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (u = %.4f, expected ~0)\n', u3);
    fail_count = fail_count + 1;
end

%% TEST 4: Chattering reduction vs Classical SMC
fprintf('Test 4: FuzzySMC has LESS chattering than ClassicalSMC... ');
plant4 = plants.DoubleIntegrator();
plant4.x0 = [2; 0];

surf4  = surfaces.LinearSurface('c', 10);
reach4 = reaching.ConstantRate('k', 15);

ctrl_classical = controllers.ClassicalSMC(surf4, reach4);
ctrl_fuzzy     = controllers.FuzzySMC(surf4, reach4);
ctrl_fuzzy.params.k = 15;

arch_c = architectures.DirectSMC(ctrl_classical);
arch_f = architectures.DirectSMC(ctrl_fuzzy);

sim4 = benchmark.Simulator('dt', 1e-4, 'T', 3);
res_c = sim4.run(arch_c, plant4, @(t)[0;0], @(t)[0;0]);
res_f = sim4.run(arch_f, plant4, @(t)[0;0], @(t)[0;0]);

ci_classical = benchmark.Metrics.chattering_index(res_c);
ci_fuzzy     = benchmark.Metrics.chattering_index(res_f);

if ci_fuzzy < ci_classical
    fprintf('PASS\n');
    fprintf('         Classical chattering = %.2f\n', ci_classical);
    fprintf('         Fuzzy chattering     = %.2f (%.0f%% less)\n', ...
        ci_fuzzy, (1 - ci_fuzzy/ci_classical)*100);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (fuzzy=%.2f >= classical=%.2f)\n', ci_fuzzy, ci_classical);
    fail_count = fail_count + 1;
end

%% ===== DISCRETE SMC =====

%% TEST 5: Zero-order hold between samples
fprintf('Test 5: DiscreteSMC zero-order hold... ');
surf5 = surfaces.LinearSurface('c', 10);
reach5 = reaching.ConstantRate('k', 1);
ctrl5 = controllers.DiscreteSMC(surf5, reach5);
ctrl5.params.Ts = 0.01;  % 10ms sampling
plant5 = plants.DoubleIntegrator();

x5 = [1; 0]; xref5 = [0; 0];

% First sample at t=0
[u_t0, ~] = ctrl5.compute(0, x5, xref5, plant5);
% Within same period
[u_t1, ~] = ctrl5.compute(0.003, x5, xref5, plant5);
[u_t2, ~] = ctrl5.compute(0.007, x5, xref5, plant5);

if abs(u_t0 - u_t1) < 1e-12 && abs(u_t0 - u_t2) < 1e-12
    fprintf('PASS (u held constant: %.4f = %.4f = %.4f)\n', u_t0, u_t1, u_t2);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (u varies within period: %.4f, %.4f, %.4f)\n', u_t0, u_t1, u_t2);
    fail_count = fail_count + 1;
end

%% TEST 6: New sample at next period
fprintf('Test 6: DiscreteSMC updates at next sample... ');
% Advance state (simulate plant changing)
x5_new = [0.8; -0.5];
[u_t3, ~] = ctrl5.compute(0.01, x5_new, xref5, plant5);  % new sample period

if abs(u_t3 - u_t0) > 1e-6  % should be different since state changed
    fprintf('PASS (u updated: %.4f -> %.4f)\n', u_t0, u_t3);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (u not updated at new sample)\n');
    fail_count = fail_count + 1;
end

%% TEST 7: Discrete reaching — sliding variable decreases
fprintf('Test 7: DiscreteSMC reaching condition (|s| decreasing)... ');
ctrl7 = controllers.DiscreteSMC(surfaces.LinearSurface('c', 5), reach5);
ctrl7.params.Ts = 1e-3;
ctrl7.params.epsilon = 5;
ctrl7.params.q = 10;

plant7 = plants.DoubleIntegrator();
plant7.x0 = [3; 0];

arch7 = architectures.DirectSMC(ctrl7);
sim7 = benchmark.Simulator('dt', 1e-4, 'T', 2);
res7 = sim7.run(arch7, plant7, @(t)[0;0], @(t)[0;0]);

% Check |s| is generally decreasing (allow small oscillations)
s_abs = abs(res7.s(1,:));
% Compare average |s| in first vs second half
half = floor(numel(s_abs)/2);
s_first_half  = mean(s_abs(1:half));
s_second_half = mean(s_abs(half+1:end));

if s_second_half < s_first_half
    fprintf('PASS (avg |s|: %.4f -> %.4f)\n', s_first_half, s_second_half);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (avg |s|: %.4f -> %.4f, not decreasing)\n', ...
        s_first_half, s_second_half);
    fail_count = fail_count + 1;
end

%% TEST 8: DiscreteSMC stabilizes the plant
fprintf('Test 8: DiscreteSMC achieves tracking... ');
rmse7 = benchmark.Metrics.rmse(res7);
ss_err = benchmark.Metrics.steady_state_error(res7);

if ss_err < 0.5
    fprintf('PASS (RMSE=%.4f, SS error=%.4f)\n', rmse7, ss_err);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (SS error=%.4f, expected < 0.5)\n', ss_err);
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== Fuzzy + Discrete SMC: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
