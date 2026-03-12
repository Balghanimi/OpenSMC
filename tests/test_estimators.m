classdef test_estimators < matlab.unittest.TestCase
    %TEST_ESTIMATORS Unit tests for all estimator implementations.

    methods (Test)

        %% === NoEstimator ===
        function test_no_estimator(testCase)
            est = estimators.NoEstimator();
            dhat = est.estimate(0, [1; 2], 0, 0);
            testCase.verifyEqual(dhat, [0; 0]);
        end

        function test_no_estimator_size(testCase)
            est = estimators.NoEstimator();
            dhat = est.estimate(0, ones(5,1), 0, 0);
            testCase.verifyEqual(numel(dhat), 5);
            testCase.verifyEqual(dhat, zeros(5,1));
        end

        %% === DisturbanceObserver ===
        function test_dob_initial(testCase)
            est = estimators.DisturbanceObserver();
            dhat = est.estimate(0, [0; 0], 0, 0);
            % Initially returns near-zero since state starts at 0
            testCase.verifyEqual(dhat(1), 0, 'AbsTol', 1e-6);
        end

        function test_dob_dimensions(testCase)
            est = estimators.DisturbanceObserver();
            dhat = est.estimate(0, zeros(4,1), [0], 0);
            testCase.verifyEqual(numel(dhat), 4);
        end

        function test_dob_reset(testCase)
            est = estimators.DisturbanceObserver();
            est.estimate(0, [0; 1], [5], 0);  % run one step
            est.reset();
            testCase.verifyEqual(est.state.dhat, 0);
            testCase.verifyEqual(est.state.omega_hat, 0);
        end

        %% === ExtendedStateObserver ===
        function test_eso_initial(testCase)
            est = estimators.ExtendedStateObserver();
            dhat = est.estimate(0, [0; 0], [0], [0]);
            testCase.verifyEqual(dhat(1), 0, 'AbsTol', 1e-6);
        end

        function test_eso_dimensions(testCase)
            est = estimators.ExtendedStateObserver();
            dhat = est.estimate(0, zeros(3,1), [0], [0]);
            testCase.verifyEqual(numel(dhat), 3);
        end

        function test_eso_reset(testCase)
            est = estimators.ExtendedStateObserver();
            est.estimate(0, [0; 0], [0], [0]);
            est.reset();
            testCase.verifyEqual(est.state.xhat, [0; 0; 0]);
        end

        function test_eso_peaking_modes(testCase)
            est_none    = estimators.ExtendedStateObserver('peaking', 'none');
            est_ramp    = estimators.ExtendedStateObserver('peaking', 'ramp');
            est_sigmoid = estimators.ExtendedStateObserver('peaking', 'sigmoid');

            % All should work without error
            est_none.estimate(0.5, [0;0], [0], [0]);
            est_ramp.estimate(0.5, [0;0], [0], [0]);
            est_sigmoid.estimate(0.5, [0;0], [0], [0]);
        end

        %% === RBF_ELM ===
        function test_rbf_elm_initial(testCase)
            est = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);
            % Initially all weights are zero => prediction = 0
            y = est.predict([0, 0]);
            testCase.verifyEqual(y, 0, 'AbsTol', 1e-10);
        end

        function test_rbf_elm_estimate(testCase)
            est = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);
            dhat = est.estimate(0, [0; 0], 0, 0);
            testCase.verifyEqual(numel(dhat), 2);
            testCase.verifyEqual(dhat(1), 0, 'AbsTol', 1e-10);
        end

        function test_rbf_elm_online_update(testCase)
            est = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);

            % Train with a simple target
            x_in = [1, 0];
            for i = 1:20
                est.online_update(x_in, 1.0);
            end

            % Prediction should move toward 1.0
            y = est.predict(x_in);
            testCase.verifyGreaterThan(abs(y), 0.1);  % learned something
        end

        function test_rbf_elm_reset(testCase)
            est = estimators.RBF_ELM('n_hidden', 10);
            est.online_update([0, 0], 5.0);
            testCase.verifyTrue(est.state.initialized);

            est.reset();
            testCase.verifyFalse(est.state.initialized);
            testCase.verifyEqual(est.state.W_out, zeros(10, 1));
        end

        function test_rbf_elm_clamping(testCase)
            est = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-1 -1], 'x_max', [1 1]);
            % Input far outside range should be clamped
            dhat = est.estimate(0, [100; -100], 0, 0);
            testCase.verifyTrue(isnumeric(dhat));
            testCase.verifyTrue(all(isfinite(dhat)));
        end

        %% === LevantDifferentiator ===
        function test_levant_initial(testCase)
            est = estimators.LevantDifferentiator('order', 2, 'L', 100);
            dhat = est.estimate(0, [0; 0], 0, [1.0]);
            testCase.verifyEqual(numel(dhat), 2);
        end

        function test_levant_dimensions(testCase)
            est = estimators.LevantDifferentiator('order', 2);
            dhat = est.estimate(0, zeros(4,1), zeros(2,1), [0]);
            testCase.verifyEqual(numel(dhat), 4);
        end

        function test_levant_reset(testCase)
            est = estimators.LevantDifferentiator('order', 2);
            est.estimate(0, [0;0], 0, [1.0]);
            est.reset();
            testCase.verifyTrue(isempty(est.state.z));
        end

        function test_levant_get_derivatives(testCase)
            est = estimators.LevantDifferentiator('order', 2);
            est.estimate(0, [0;0], 0, [5.0]);
            derivs = est.get_derivatives();
            testCase.verifyEqual(numel(derivs), 3);  % [f, fdot, fddot]
        end

        function test_levant_orders(testCase)
            % Test different orders initialize correctly
            for ord = 1:4
                est = estimators.LevantDifferentiator('order', ord);
                testCase.verifyEqual(est.params.order, ord);
                testCase.verifyEqual(numel(est.params.lambdas), ord+1);
            end
        end

        %% === All Estimators ===
        function test_all_estimators_describe(testCase)
            ests = {
                estimators.NoEstimator(), ...
                estimators.DisturbanceObserver(), ...
                estimators.ExtendedStateObserver(), ...
                estimators.RBF_ELM(), ...
                estimators.LevantDifferentiator()
            };
            for i = 1:numel(ests)
                info = ests{i}.describe();
                testCase.verifyTrue(isfield(info, 'name'));
            end
        end
    end
end
