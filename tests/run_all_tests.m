%% OpenSMC Test Runner
%
%  Run all unit tests and display results.
%  Execute from the OpenSMC root directory:
%    >> cd D:/OpenSMC
%    >> run('tests/run_all_tests.m')

clear; clc;
addpath('..');

fprintf('=== OpenSMC Test Suite ===\n\n');

import matlab.unittest.TestSuite;
import matlab.unittest.TestRunner;
import matlab.unittest.plugins.DiagnosticsValidationPlugin;

% Collect all test classes
suite = [
    TestSuite.fromClass(?test_surfaces), ...
    TestSuite.fromClass(?test_reaching), ...
    TestSuite.fromClass(?test_plants), ...
    TestSuite.fromClass(?test_controllers), ...
    TestSuite.fromClass(?test_estimators), ...
    TestSuite.fromClass(?test_benchmark), ...
    TestSuite.fromClass(?test_integration)
];

% Run
runner = TestRunner.withTextOutput('Verbosity', 3);
results = runner.run(suite);

% Summary
fprintf('\n=== Summary ===\n');
fprintf('Total:  %d\n', numel(results));
fprintf('Passed: %d\n', sum([results.Passed]));
fprintf('Failed: %d\n', sum([results.Failed]));
fprintf('Errors: %d\n', sum([results.Incomplete]));

if all([results.Passed])
    fprintf('\nAll tests passed!\n');
else
    fprintf('\nFailed tests:\n');
    failed = results(~[results.Passed]);
    for i = 1:numel(failed)
        fprintf('  - %s\n', failed(i).Name);
    end
end
