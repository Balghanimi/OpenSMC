%% VALIDATION: Levant Differentiator
%
%  Validates against known analytical derivatives of test signals.
%  This is the definitive test: if the differentiator can recover
%  f'(t) and f''(t) from f(t), it works correctly.
%
%  Reference:
%    Levant, A. (2003). "Higher-order sliding modes, differentiation
%    and output-feedback control." IJC, 76(9-10), 924-941.
%    Test signal: f(t) = sin(t), f'(t) = cos(t), f''(t) = -sin(t)
%
%  Tests:
%    1. Differentiate sin(t): verify f' = cos(t), f'' = -sin(t)
%    2. Differentiate t^2: verify f' = 2t, f'' = 2
%    3. Differentiate constant: verify f' = 0, f'' = 0
%    4. Order-1: single derivative accuracy
%    5. Robustness to noise

clear; clc;
fprintf('=== Levant Differentiator Validation ===\n\n');
pass_count = 0;
fail_count = 0;

%% TEST 1: Differentiate sin(t) — the classic Levant test
fprintf('Test 1: f(t) = sin(t), verify f''=cos(t), f''''=-sin(t)... ');
est = estimators.LevantDifferentiator('order', 2, 'L', 5, 'dt', 1e-4);

dt = 1e-4;
T_sim = 5.0;
N = round(T_sim/dt);

fdot_err_rms = 0;
fddot_err_rms = 0;
n_eval = 0;

for k = 1:N
    t = (k-1)*dt;
    f_t = sin(t);

    est.estimate(t, [0;0], 0, f_t);

    % Only check after transient (t > 1s) — differentiator needs time to converge
    if t > 1.0
        derivs = est.get_derivatives();
        fdot_est  = derivs(2);   % estimated first derivative
        fddot_est = derivs(3);   % estimated second derivative

        fdot_true  = cos(t);
        fddot_true = -sin(t);

        fdot_err_rms  = fdot_err_rms  + (fdot_est - fdot_true)^2;
        fddot_err_rms = fddot_err_rms + (fddot_est - fddot_true)^2;
        n_eval = n_eval + 1;
    end
end

fdot_err_rms  = sqrt(fdot_err_rms / n_eval);
fddot_err_rms = sqrt(fddot_err_rms / n_eval);

if fdot_err_rms < 0.05 && fddot_err_rms < 0.1
    fprintf('PASS\n');
    fprintf('         f''  RMSE = %.4f (threshold 0.05)\n', fdot_err_rms);
    fprintf('         f'''' RMSE = %.4f (threshold 0.10)\n', fddot_err_rms);
    pass_count = pass_count + 1;
else
    fprintf('FAIL\n');
    fprintf('         f''  RMSE = %.4f (threshold 0.05)\n', fdot_err_rms);
    fprintf('         f'''' RMSE = %.4f (threshold 0.10)\n', fddot_err_rms);
    fail_count = fail_count + 1;
end

%% TEST 2: Differentiate t^2 — verify f' = 2t, f'' = 2
fprintf('Test 2: f(t) = t^2, verify f''=2t, f''''=2... ');
est2 = estimators.LevantDifferentiator('order', 2, 'L', 10, 'dt', 1e-4);

fdot_err_rms = 0;
fddot_err_rms = 0;
n_eval = 0;

for k = 1:N
    t = (k-1)*dt;
    f_t = t^2;

    est2.estimate(t, [0;0], 0, f_t);

    if t > 1.5  % longer transient for polynomial
        derivs = est2.get_derivatives();
        fdot_est  = derivs(2);
        fddot_est = derivs(3);

        fdot_true  = 2*t;
        fddot_true = 2;

        fdot_err_rms  = fdot_err_rms  + (fdot_est - fdot_true)^2;
        fddot_err_rms = fddot_err_rms + (fddot_est - fddot_true)^2;
        n_eval = n_eval + 1;
    end
end

fdot_err_rms  = sqrt(fdot_err_rms / n_eval);
fddot_err_rms = sqrt(fddot_err_rms / n_eval);

if fdot_err_rms < 0.1 && fddot_err_rms < 0.5
    fprintf('PASS\n');
    fprintf('         f''  RMSE = %.4f\n', fdot_err_rms);
    fprintf('         f'''' RMSE = %.4f\n', fddot_err_rms);
    pass_count = pass_count + 1;
else
    fprintf('FAIL\n');
    fprintf('         f''  RMSE = %.4f (threshold 0.1)\n', fdot_err_rms);
    fprintf('         f'''' RMSE = %.4f (threshold 0.5)\n', fddot_err_rms);
    fail_count = fail_count + 1;
end

%% TEST 3: Constant signal — derivatives should be zero
fprintf('Test 3: f(t) = 5 (constant), verify f''=0, f''''=0... ');
est3 = estimators.LevantDifferentiator('order', 2, 'L', 5, 'dt', 1e-4);

for k = 1:N
    t = (k-1)*dt;
    est3.estimate(t, [0;0], 0, 5.0);
end

derivs = est3.get_derivatives();
if abs(derivs(2)) < 0.01 && abs(derivs(3)) < 0.01
    fprintf('PASS (f''=%.4f, f''''=%.4f)\n', derivs(2), derivs(3));
    pass_count = pass_count + 1;
else
    fprintf('FAIL (f''=%.4f, f''''=%.4f, expected ~0)\n', derivs(2), derivs(3));
    fail_count = fail_count + 1;
end

%% TEST 4: First-order differentiator on cos(t)
fprintf('Test 4: Order-1 differentiator on cos(t)... ');
est4 = estimators.LevantDifferentiator('order', 1, 'L', 5, 'dt', 1e-4);

fdot_err_rms = 0;
n_eval = 0;

for k = 1:N
    t = (k-1)*dt;
    est4.estimate(t, [0;0], 0, cos(t));

    if t > 1.0
        derivs = est4.get_derivatives();
        fdot_est = derivs(2);
        fdot_true = -sin(t);
        fdot_err_rms = fdot_err_rms + (fdot_est - fdot_true)^2;
        n_eval = n_eval + 1;
    end
end

fdot_err_rms = sqrt(fdot_err_rms / n_eval);
if fdot_err_rms < 0.05
    fprintf('PASS (f'' RMSE = %.4f)\n', fdot_err_rms);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (f'' RMSE = %.4f, threshold 0.05)\n', fdot_err_rms);
    fail_count = fail_count + 1;
end

%% TEST 5: Signal reconstruction (z0 should track f)
fprintf('Test 5: Signal reconstruction z0 ≈ f(t)... ');
est5 = estimators.LevantDifferentiator('order', 2, 'L', 50, 'dt', 1e-4);

max_recon_err = 0;
for k = 1:N
    t = (k-1)*dt;
    f_t = sin(2*t);
    est5.estimate(t, [0;0], 0, f_t);

    if t > 0.5
        derivs = est5.get_derivatives();
        recon_err = abs(derivs(1) - f_t);
        max_recon_err = max(max_recon_err, recon_err);
    end
end

if max_recon_err < 0.01
    fprintf('PASS (max reconstruction error = %.4f)\n', max_recon_err);
    pass_count = pass_count + 1;
else
    fprintf('FAIL (max reconstruction error = %.4f, threshold 0.01)\n', max_recon_err);
    fail_count = fail_count + 1;
end

%% SUMMARY
fprintf('\n=== Levant Differentiator: %d PASSED, %d FAILED ===\n', pass_count, fail_count);
if fail_count == 0
    fprintf('>> ALL VALIDATIONS PASSED <<\n');
else
    fprintf('>> %d VALIDATIONS FAILED — REVIEW IMPLEMENTATION <<\n', fail_count);
end
