%% MASTER VALIDATION: Run All New Module Validations
%
%  Executes ALL validation scripts for the 9 new OpenSMC modules.
%  Each script tests against analytical solutions, theoretical guarantees,
%  and published reference values.
%
%  ALL CODE IS ORIGINAL — written from textbook/paper math formulations.
%  Validation proves correctness by comparing against:
%    - Known analytical solutions (M(q) values, torque formula, derivatives)
%    - Theoretical guarantees (T_max bound, s(0)=0, convergence before Tc)
%    - Physics principles (energy conservation, equilibrium conditions)
%    - Cross-validation (computed-torque tracking, chattering comparison)
%
%  HOW TO INTERPRET:
%    "PASS" = our code produces the SAME result as the published math
%    "FAIL" = our code DISAGREES with the published math => BUG
%
%  Run this script in MATLAB from the OpenSMC root directory:
%    >> cd D:/OpenSMC
%    >> run('tests/validate_all_new_modules.m')

clc;
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║   OpenSMC: Master Validation Suite              ║\n');
fprintf('║   9 New Modules, %s                     ║\n', datestr(now, 'yyyy-mm-dd'));
fprintf('╚══════════════════════════════════════════════════╝\n\n');

addpath('tests');
total_start = tic;

%% 1. TwoLinkArm Plant (6 tests)
fprintf('--- [1/5] TwoLinkArm Plant ---\n');
try
    validate_twolinkarm;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
fprintf('\n');

%% 2. PMSM Motor Plant (7 tests)
fprintf('--- [2/5] PMSM Motor Plant ---\n');
try
    validate_pmsm;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
fprintf('\n');

%% 3. Levant Differentiator (5 tests)
fprintf('--- [3/5] Levant Differentiator ---\n');
try
    validate_levant;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
fprintf('\n');

%% 4. Fixed-Time SMC Controller (6 tests)
fprintf('--- [4/5] Fixed-Time SMC Controller ---\n');
try
    validate_fixedtime;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
fprintf('\n');

%% 5. Surfaces + Fuzzy + Discrete (16 tests)
fprintf('--- [5/5] Surfaces, Fuzzy SMC, Discrete SMC ---\n');
try
    validate_surfaces;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
fprintf('\n');
try
    validate_fuzzy_discrete;
catch ME
    fprintf('ERROR: %s\n', ME.message);
end

%% Summary
try elapsed = toc(total_start); catch, elapsed = NaN; end
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║   Validation Complete in %.1f seconds           ║\n', elapsed);
fprintf('╚══════════════════════════════════════════════════╝\n');
fprintf('\nReview each section above.\n');
fprintf('ALL tests must show PASS for the implementation to be correct.\n');
fprintf('Any FAIL indicates a bug that must be fixed before publication.\n');
