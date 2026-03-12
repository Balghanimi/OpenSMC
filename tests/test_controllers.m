classdef test_controllers < matlab.unittest.TestCase
    %TEST_CONTROLLERS Unit tests for all controller implementations.

    methods (Test)

        %% === ClassicalSMC ===
        function test_classical_basic(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            x    = [1; 0];
            xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
            testCase.verifyTrue(isfield(info, 'dhat'));
            % s = edot + c*e = 0 + 5*(-1) = -5
            testCase.verifyEqual(info.s, -5, 'AbsTol', 1e-10);
        end

        function test_classical_with_estimator(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            est   = estimators.NoEstimator();
            ctrl  = controllers.ClassicalSMC(surf, reach, est);
            plant = plants.DoubleIntegrator();

            [u, info] = ctrl.compute(0, [0;0], [1;0], plant);
            testCase.verifyTrue(isnumeric(u));
            testCase.verifyEqual(info.dhat, [0; 0]);
        end

        function test_classical_reset(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.ClassicalSMC(surf, reach);

            ctrl.state.eint = 999;
            ctrl.reset();
            testCase.verifyEqual(ctrl.state.eint, 0);
        end

        function test_classical_describe(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.ClassicalSMC(surf, reach);

            info = ctrl.describe();
            testCase.verifyEqual(info.name, 'ClassicalSMC');
            testCase.verifyTrue(isfield(info, 'surface'));
            testCase.verifyTrue(isfield(info, 'reaching'));
        end

        %% === AggregatedHSMC ===
        function test_agg_hsmc_basic(testCase)
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);
            ctrl = controllers.AggregatedHSMC(dummy_s, dummy_r);
            crane = plants.SinglePendulumCrane();

            xref = [2; 0; 0; 0];
            [u, info] = ctrl.compute(0, crane.x0, xref, crane);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));   % main surface
            testCase.verifyTrue(isfield(info, 's1'));
            testCase.verifyTrue(isfield(info, 's2'));
            testCase.verifyTrue(isfield(info, 'S'));
            testCase.verifyEqual(info.s, info.S);      % s aliases S
        end

        %% === IncrementalHSMC ===
        function test_inc_hsmc_basic(testCase)
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);
            ctrl = controllers.IncrementalHSMC(dummy_s, dummy_r);
            crane = plants.SinglePendulumCrane();

            xref = [2; 0; 0; 0];
            [u, info] = ctrl.compute(0, crane.x0, xref, crane);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));   % main surface
            testCase.verifyTrue(isfield(info, 's1'));
            testCase.verifyTrue(isfield(info, 's2'));
            testCase.verifyTrue(isfield(info, 's3'));
            testCase.verifyEqual(info.s, info.s3);     % s aliases s3
        end

        function test_inc_hsmc_sign_switching(testCase)
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);
            ctrl = controllers.IncrementalHSMC(dummy_s, dummy_r);
            crane = plants.SinglePendulumCrane();

            % x3*s1 < 0 should flip c2 sign
            x_neg = [0; 0; -0.1; 0];  % x3 < 0
            xref  = [2; 0; 0; 0];     % s1 = 0.85*2 + 0 = 1.7 > 0
            % x3*s1 = -0.1*1.7 < 0 => c2 should be negative
            [~, info] = ctrl.compute(0, x_neg, xref, crane);
            testCase.verifyLessThan(info.c2, 0);
        end

        %% === CombiningHSMC ===
        function test_cmb_hsmc_basic(testCase)
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);
            ctrl = controllers.CombiningHSMC(dummy_s, dummy_r);
            crane = plants.SinglePendulumCrane();

            xref = [2; 0; 0; 0];
            [u, info] = ctrl.compute(0, crane.x0, xref, crane);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
            testCase.verifyTrue(isfield(info, 'z'));
            testCase.verifyTrue(isfield(info, 'zdot'));
        end

        %% === ITSMC ===
        function test_itsmc_basic(testCase)
            surf  = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1, 'p', 5, 'q', 7);
            reach = reaching.Saturation('k', 0.5, 'phi', 0.2);
            est   = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);
            ctrl  = controllers.ITSMC(surf, reach, est);

            % Create SISO plant wrapper
            plant = struct();
            plant.n_inputs = 1;
            plant.n_states = 2;
            plant.get_error = @(x, xref) deal(xref(1)-x(1), xref(2)-x(2));
            plant.output = @(x) x(1);

            x = [0.5; 0]; xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
            testCase.verifyTrue(isfield(info, 'dhat'));
            testCase.verifyTrue(isfield(info, 'e'));
        end

        function test_itsmc_reset(testCase)
            surf  = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1);
            reach = reaching.Saturation('k', 0.5, 'phi', 0.2);
            ctrl  = controllers.ITSMC(surf, reach);

            ctrl.state.e_integral = 100;
            ctrl.reset();
            testCase.verifyEqual(ctrl.state.e_integral, 0);
            testCase.verifyEqual(ctrl.state.initialized, false);
        end

        %% === NFTSMC ===
        function test_nftsmc_basic(testCase)
            surf  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
            reach = reaching.PowerRate('k', 80, 'alpha', 5/7);
            ctrl  = controllers.NFTSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            % Configure
            ctrl.params.input_gain = -1;
            ctrl.params.alpha = 500;
            ctrl.params.beta  = 120;

            x = [0.5; 0]; xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
            testCase.verifyTrue(isfield(info, 'e'));
            testCase.verifyTrue(isfield(info, 'edot'));
            testCase.verifyTrue(isfield(info, 'sig_e'));
            testCase.verifyTrue(isfield(info, 'uint'));
        end

        function test_nftsmc_on_nanopositioner(testCase)
            nano  = plants.DualStageNanopositioner();
            surf  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
            reach = reaching.PowerRate('k', 80, 'alpha', 5/7);
            ctrl  = controllers.NFTSMC(surf, reach);

            ctrl.params.input_gain = nano.get_input_gain();
            ctrl.params.dt = 1e-5;

            x = zeros(4, 1); xref = [1e-6; 0; 0; 0];
            [u, info] = ctrl.compute(0, x, xref, nano);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyEqual(numel(u), 1);
            % Control should be bounded
            testCase.verifyLessThanOrEqual(abs(u), ctrl.params.u_max);
            % Should have positive error (reference > output)
            testCase.verifyGreaterThan(info.e, 0);
        end

        function test_nftsmc_reset(testCase)
            surf  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
            reach = reaching.PowerRate('k', 80, 'alpha', 5/7);
            ctrl  = controllers.NFTSMC(surf, reach);

            ctrl.state.e_integral = 999;
            ctrl.state.e_prev = 0.5;
            ctrl.reset();
            testCase.verifyEqual(ctrl.state.e_integral, 0);
            testCase.verifyEqual(ctrl.state.e_prev, 0);
        end

        function test_nftsmc_saturation(testCase)
            surf  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
            reach = reaching.PowerRate('k', 80, 'alpha', 5/7);
            ctrl  = controllers.NFTSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            ctrl.params.input_gain = -1;
            ctrl.params.u_max = 5;

            % Large error should trigger saturation
            x = [10; 0]; xref = [0; 0];
            [u, ~] = ctrl.compute(0, x, xref, plant);
            testCase.verifyLessThanOrEqual(abs(u), 5);
        end

        %% === FuzzySMC ===
        function test_fuzzy_basic(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FuzzySMC(surf, reach);
            plant = plants.DoubleIntegrator();

            x = [1; 0]; xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
            testCase.verifyTrue(isfield(info, 'k_fuzzy'));
        end

        function test_fuzzy_zero_error(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FuzzySMC(surf, reach);
            plant = plants.DoubleIntegrator();

            [u, info] = ctrl.compute(0, [0;0], [0;0], plant);
            % At zero error, s=0 => fuzzy output ~0 => u ~0
            testCase.verifyEqual(u, 0, 'AbsTol', 1);
        end

        function test_fuzzy_reset(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FuzzySMC(surf, reach);
            ctrl.reset();
            testCase.verifyEqual(ctrl.state.s_prev, 0);
        end

        %% === DiscreteSMC ===
        function test_discrete_basic(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.DiscreteSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            x = [1; 0]; xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 's'));
        end

        function test_discrete_zero_order_hold(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.DiscreteSMC(surf, reach);
            ctrl.params.Ts = 0.01;
            plant = plants.DoubleIntegrator();

            x = [1; 0]; xref = [0; 0];
            [u1, ~] = ctrl.compute(0, x, xref, plant);
            % Within same sampling period, output should be held
            [u2, ~] = ctrl.compute(0.005, x, xref, plant);
            testCase.verifyEqual(u1, u2, 'AbsTol', 1e-10);
        end

        function test_discrete_reset(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.DiscreteSMC(surf, reach);
            ctrl.reset();
            testCase.verifyEqual(ctrl.state.t_last, -inf);
        end

        %% === FixedTimeSMC ===
        function test_fixedtime_basic(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FixedTimeSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            x = [1; 0]; xref = [0; 0];
            [u, info] = ctrl.compute(0, x, xref, plant);

            testCase.verifyTrue(isnumeric(u));
            testCase.verifyTrue(isfield(info, 'T_max'));
            testCase.verifyGreaterThan(info.T_max, 0);
        end

        function test_fixedtime_settling_bound(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FixedTimeSMC(surf, reach);
            ctrl.params.alpha = 10;
            ctrl.params.beta = 10;
            ctrl.params.p = 0.5;
            ctrl.params.q = 1.5;

            T = ctrl.get_max_settling_time();
            expected = 1/(10*0.5) + 1/(10*0.5);
            testCase.verifyEqual(T, expected, 'AbsTol', 1e-10);
        end

        function test_fixedtime_zero_error(testCase)
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.ConstantRate('k', 10);
            ctrl  = controllers.FixedTimeSMC(surf, reach);
            plant = plants.DoubleIntegrator();

            [u, ~] = ctrl.compute(0, [0;0], [0;0], plant);
            testCase.verifyEqual(u, 0, 'AbsTol', 1e-10);
        end

        %% === All Controllers ===
        function test_all_controllers_have_name(testCase)
            dummy_s = surfaces.LinearSurface();
            dummy_r = reaching.ConstantRate();

            ctrls = {
                controllers.ClassicalSMC(dummy_s, dummy_r), ...
                controllers.AggregatedHSMC(dummy_s, dummy_r), ...
                controllers.IncrementalHSMC(dummy_s, dummy_r), ...
                controllers.CombiningHSMC(dummy_s, dummy_r), ...
                controllers.ITSMC(dummy_s, dummy_r), ...
                controllers.NFTSMC(dummy_s, dummy_r), ...
                controllers.FuzzySMC(dummy_s, dummy_r), ...
                controllers.DiscreteSMC(dummy_s, dummy_r), ...
                controllers.FixedTimeSMC(dummy_s, dummy_r)
            };
            for i = 1:numel(ctrls)
                testCase.verifyNotEmpty(ctrls{i}.name);
            end
        end
    end
end
