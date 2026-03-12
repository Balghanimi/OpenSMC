%% VALIDATION: Global Surface & Predefined-Time Surface
%
%  Tests the KEY theoretical guarantees that define these surfaces.
%  If these properties don't hold, the surface is useless.
%
%  GlobalSurface: s(0) = 0 ALWAYS (the defining property)
%  PredefinedTimeSurface: error -> 0 before t = Tc
%
%  Tests:
%    1. GlobalSurface: s(0) = 0 for 100 random initial conditions
%    2. GlobalSurface: converges to linear surface as t -> inf
%    3. GlobalSurface: full closed-loop simulation reaches sliding from t=0
%    4. PredefinedTimeSurface: gain diverges as t -> Tc (forcing convergence)
%    5. PredefinedTimeSurface: reverts to linear after Tc
%    6. PredefinedTimeSurface: full simulation — error < threshold at t=Tc

clear; clc;
fprintf('=== Surface Validation (Global + Predefined-Time) ===\n\n');
pass_count = 0;
fail_count = 0;

%% ===== GLOBAL SURFACE =====

%% TEST 1: s(0) = 0 for 100 random initial errors
fprintf('Test 1: GlobalSurface s(0) = 0 for random ICs... ');
test1_pass = true;
for trial = 1:100
    surf = surfaces.GlobalSurface('c', rand*20, 'alpha', rand*10);
    e0    = (rand-0.5)*20;
    edot0 = (rand-0.5)*10;
    s0 = surf.compute(e0, edot0, 0, 0);  % t=0

    if abs(s0) > 1e-12
        test1_pass = false;
        fprintf('FAIL at trial %d: s(0) = %.2e (e=%.2f, edot=%.2f)\n', ...
            trial, s0, e0, edot0);
        break;
    end
end
if test1_pass
    fprintf('PASS (100 random ICs, all s(0) = 0)\n');
    pass_count = pass_count + 1;
else
    fail_count = fail_count + 1;
end

%% TEST 2: s(0) = 0 for vector errors
fprintf('Test 2: GlobalSurface s(0) = 0 for vector errors... ');
surf2 = surfaces.GlobalSurface('c', 10, 'alpha', 5);
e_vec    = [3; -2; 0.5];
edot_vec = [-1; 4; 2];
s0_vec = surf2.compute(e_vec, edot_vec, zeros(3,1), 0);

if norm(s0_vec) < 1e-12
    fprintf('PASS (3-element vector, ||s(0)|| = %.2e)\n', norm(s0_vec));
    pass_count = pass_count + 1;
else
    fprintf('FAIL (||s(0)|| = %.2e)\n', norm(s0_vec));
    fail_count = fail_count + 1;
end

%% TEST 3: Convergence to linear surface
fprintf('Test 3: GlobalSurface -> LinearSurface as t -> inf... ');
c_test = 8;
surf3g = surfaces.GlobalSurface('c', c_test, 'alpha', 3);
surf3l = surfaces.LinearSurface('c', c_test);

e_test = 1.5;
edot_test = -0.5;

% Initialize global surface
surf3g.compute(e_test, edot_test, 0, 0);

% At t=20 (large), should match linear
s_global = surf3g.compute(e_test, edot_test, 0, 20);
s_linear = surf3l.compute(e_test, edot_test, 0, 20);

err = abs(s_global - s_linear);
if err < 1e-6
    fprintf('PASS (difference = %.2e at t=20)\n', err);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (difference = %.2e)\n', err);
    fail_count = fail_count + 1;
end

%% TEST 4: Full closed-loop — sliding variable stays near 0 from start
fprintf('Test 4: GlobalSurface closed-loop: s(t) ≈ 0 from t=0... ');
plant = plants.DoubleIntegrator();
plant.x0 = [5; 0];  % large initial error

surf4 = surfaces.GlobalSurface('c', 10, 'alpha', 5);
reach4 = reaching.SuperTwisting('k1', 15, 'k2', 10);
ctrl4 = controllers.ClassicalSMC(surf4, reach4);
arch4 = architectures.DirectSMC(ctrl4);

sim = benchmark.Simulator('dt', 1e-4, 'T', 3);
result4 = sim.run(arch4, plant, @(t)[0;0], @(t)[0;0]);

% s should start near 0
s_initial = abs(result4.s(1, 1));
s_max_first_100ms = max(abs(result4.s(1, 1:1000)));  % first 0.1s

% For GlobalSurface, the KEY guarantee is s(0)=0 (already tested above).
% In closed-loop, s stays smaller than with LinearSurface (no reaching phase).
% Compare against LinearSurface on same plant to validate the advantage.
surf_lin = surfaces.LinearSurface('c', 10);
ctrl_lin = controllers.ClassicalSMC(surf_lin, reach4);
arch_lin = architectures.DirectSMC(ctrl_lin);
plant_lin = plants.DoubleIntegrator();
plant_lin.x0 = [5; 0];
res_lin = sim.run(arch_lin, plant_lin, @(t)[0;0], @(t)[0;0]);

s_max_lin  = max(abs(res_lin.s(1, 1:1000)));
s_max_glob = s_max_first_100ms;

if s_initial < 0.01 && s_max_glob < s_max_lin
    fprintf('PASS\n');
    fprintf('         s(0) = %.6f (exact zero)\n', s_initial);
    fprintf('         Global max|s| in [0,0.1s] = %.2f\n', s_max_glob);
    fprintf('         Linear max|s| in [0,0.1s] = %.2f (worse)\n', s_max_lin);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (s(0)=%.4f, Global=%.2f, Linear=%.2f)\n', ...
        s_initial, s_max_glob, s_max_lin);
    fail_count = fail_count + 1;
end

%% ===== PREDEFINED-TIME SURFACE =====

%% TEST 5: Gain increases as t -> Tc
fprintf('Test 5: PredefinedTime gain increases toward Tc... ');
Tc = 2.0;
surf5 = surfaces.PredefinedTimeSurface('Tc', Tc, 'c_inf', 10);

s_at_01 = surf5.compute(1, 0, 0, 0.1);
s_at_05 = surf5.compute(1, 0, 0, 0.5);
s_at_10 = surf5.compute(1, 0, 0, 1.0);
s_at_19 = surf5.compute(1, 0, 0, 1.9);

% Gain should be monotonically increasing
if abs(s_at_05) > abs(s_at_01) && ...
   abs(s_at_10) > abs(s_at_05) && ...
   abs(s_at_19) > abs(s_at_10)
    fprintf('PASS (s: %.2f -> %.2f -> %.2f -> %.2f)\n', ...
        s_at_01, s_at_05, s_at_10, s_at_19);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (gain not monotonically increasing)\n');
    fail_count = fail_count + 1;
end

%% TEST 6: After Tc, reverts to linear surface
fprintf('Test 6: PredefinedTime -> Linear after Tc... ');
surf6p = surfaces.PredefinedTimeSurface('Tc', 2.0, 'c_inf', 10);
surf6l = surfaces.LinearSurface('c', 10);

s_predef = surf6p.compute(1, 2, 0, 5.0);  % t = 5 > Tc = 2
s_linear = surf6l.compute(1, 2, 0, 5.0);

if abs(s_predef - s_linear) < 1e-12
    fprintf('PASS (both = %.4f at t=5)\n', s_predef);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (predef=%.4f, linear=%.4f)\n', s_predef, s_linear);
    fail_count = fail_count + 1;
end

%% TEST 7: Known gain value at t=0
%  At t=0: c(0) = pi/(2*Tc) / cos(0) = pi/(2*Tc)
fprintf('Test 7: PredefinedTime gain at t=0 = pi/(2*Tc)... ');
Tc7 = 3.0;
surf7 = surfaces.PredefinedTimeSurface('Tc', Tc7);
s7 = surf7.compute(1, 0, 0, 0);  % e=1, edot=0 => s = c(0)*1

c0_expected = pi / (2*Tc7);
if abs(s7 - c0_expected) < 1e-12
    fprintf('PASS (c(0) = %.6f = pi/(2*%.1f))\n', s7, Tc7);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (s=%.6f, expected %.6f)\n', s7, c0_expected);
    fail_count = fail_count + 1;
end

%% TEST 8: Full closed-loop — error converges before Tc
fprintf('Test 8: PredefinedTime closed-loop: |e(Tc)| < threshold... ');
plant8 = plants.DoubleIntegrator();
plant8.x0 = [3; 0];  % start at x=3

Tc8 = 2.0;
surf8 = surfaces.PredefinedTimeSurface('Tc', Tc8, 'c_inf', 15);
reach8 = reaching.SuperTwisting('k1', 20, 'k2', 15);
ctrl8 = controllers.ClassicalSMC(surf8, reach8);
arch8 = architectures.DirectSMC(ctrl8);

sim8 = benchmark.Simulator('dt', 1e-4, 'T', 4);
result8 = sim8.run(arch8, plant8, @(t)[0;0], @(t)[0;0]);

% Find error at t=Tc
tc_idx = find(result8.t >= Tc8, 1, 'first');
e_at_Tc = abs(result8.e(1, tc_idx));
e_ss = mean(abs(result8.e(1, end-1000:end)));

fprintf('\n         |e(Tc)| = %.4f, |e_ss| = %.6f\n', e_at_Tc, e_ss);
if e_at_Tc < 0.5 && e_ss < 0.1
    fprintf('         PASS\n');
    pass_count = pass_count + 1;
else
    fprintf('         FAIL (e_at_Tc=%.4f, threshold 0.5)\n', e_at_Tc);
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== Surfaces: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
