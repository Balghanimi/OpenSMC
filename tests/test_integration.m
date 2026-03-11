classdef test_integration < matlab.unittest.TestCase
    %TEST_INTEGRATION Full pipeline integration tests.
    %
    %   Tests that complete control pipelines (surface + reaching +
    %   estimator + controller + architecture + plant + simulator)
    %   work end-to-end without errors.

    methods (Test)

        %% === Double Integrator Pipeline ===
        function test_di_all_surfaces(testCase)
            % Test all 5 basic surfaces on double integrator
            plant   = plants.DoubleIntegrator();
            reach   = reaching.Saturation('k', 20, 'phi', 0.1);
            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);
            sim     = benchmark.Simulator('dt', 1e-3, 'T', 2);

            surface_list = {
                surfaces.LinearSurface('c', 10), ...
                surfaces.TerminalSurface('beta', 10, 'p', 5, 'q', 7), ...
                surfaces.NonsingularTerminalSurface('beta', 10, 'p', 7, 'q', 5), ...
                surfaces.FastTerminalSurface('alpha', 2, 'beta', 1, 'p', 9, 'q', 5), ...
                surfaces.IntegralTerminalSurface('c1', 10, 'c2', 5, 'p', 5, 'q', 7)
            };

            for i = 1:numel(surface_list)
                ctrl = controllers.ClassicalSMC(surface_list{i}, reach);
                arch = architectures.DirectSMC(ctrl);
                result = sim.run(arch, plant, ref_fn, dist_fn);

                testCase.verifyTrue(all(isfinite(result.x(:))), ...
                    sprintf('Surface %d produced NaN/Inf', i));

                m = benchmark.Metrics.compute_all(result);
                testCase.verifyTrue(isfinite(m.rmse), ...
                    sprintf('Surface %d: RMSE not finite', i));
            end
        end

        function test_di_all_reaching(testCase)
            % Test all 5 reaching laws on double integrator
            plant   = plants.DoubleIntegrator();
            surf    = surfaces.LinearSurface('c', 10);
            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);
            sim     = benchmark.Simulator('dt', 1e-3, 'T', 2);

            reach_list = {
                reaching.ConstantRate('k', 20), ...
                reaching.ExponentialRate('k', 10, 'q', 5), ...
                reaching.PowerRate('k', 15, 'alpha', 0.5), ...
                reaching.SuperTwisting('k1', 15, 'k2', 10), ...
                reaching.Saturation('k', 20, 'phi', 0.1)
            };

            for i = 1:numel(reach_list)
                ctrl = controllers.ClassicalSMC(surf, reach_list{i});
                arch = architectures.DirectSMC(ctrl);
                result = sim.run(arch, plant, ref_fn, dist_fn);

                testCase.verifyTrue(all(isfinite(result.x(:))), ...
                    sprintf('Reaching %d produced NaN/Inf', i));
            end
        end

        function test_di_with_disturbance(testCase)
            plant   = plants.DoubleIntegrator();
            surf    = surfaces.LinearSurface('c', 10);
            reach   = reaching.ConstantRate('k', 30);
            ctrl    = controllers.ClassicalSMC(surf, reach);
            arch    = architectures.DirectSMC(ctrl);
            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.step(0.5, 1, 2);
            sim     = benchmark.Simulator('dt', 1e-3, 'T', 3);

            result = sim.run(arch, plant, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));
        end

        function test_di_with_dob(testCase)
            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 15);
            est   = estimators.DisturbanceObserver('dt', 1e-3);
            ctrl  = controllers.ClassicalSMC(surf, reach, est);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.step(0.5, 1, 2);
            sim     = benchmark.Simulator('dt', 1e-3, 'T', 2);

            result = sim.run(arch, plant, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));
        end

        function test_di_with_eso(testCase)
            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 15);
            est   = estimators.ExtendedStateObserver('dt', 1e-3);
            ctrl  = controllers.ClassicalSMC(surf, reach, est);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);
            sim     = benchmark.Simulator('dt', 1e-3, 'T', 2);

            result = sim.run(arch, plant, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));
        end

        %% === Crane Pipeline ===
        function test_crane_aggregated(testCase)
            crane   = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);

            ctrl = controllers.AggregatedHSMC(dummy_s, dummy_r);
            arch = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.transport(2, 4);
            dist_fn = utils.disturbances.none(4);
            sim     = benchmark.Simulator('dt', 1e-4, 'T', 4);

            result = sim.run(arch, crane, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));

            % Trolley should move toward target
            testCase.verifyGreaterThan(result.x(1, end), 0.5);
        end

        function test_crane_incremental(testCase)
            crane   = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);

            ctrl = controllers.IncrementalHSMC(dummy_s, dummy_r);
            arch = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.transport(2, 4);
            dist_fn = utils.disturbances.none(4);
            sim     = benchmark.Simulator('dt', 1e-4, 'T', 4);

            result = sim.run(arch, crane, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));
        end

        function test_crane_combining(testCase)
            crane   = plants.SinglePendulumCrane('M', 37.32, 'm', 5, 'l', 1.05);
            dummy_s = surfaces.LinearSurface('c', 1);
            dummy_r = reaching.ConstantRate('k', 1);

            ctrl = controllers.CombiningHSMC(dummy_s, dummy_r);
            arch = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.transport(2, 4);
            dist_fn = utils.disturbances.none(4);
            sim     = benchmark.Simulator('dt', 1e-4, 'T', 4);

            result = sim.run(arch, crane, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))));
        end

        %% === Quadrotor Pipeline ===
        function test_quadrotor_itsmc(testCase)
            quad = plants.Quadrotor6DOF();
            dt   = 1e-3;

            dummy_s = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1, 'p', 5, 'q', 7);
            dummy_r = reaching.Saturation('k', 0.5, 'phi', 0.2);

            est_x = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);
            est_y = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);
            est_z = estimators.RBF_ELM('n_hidden', 10, 'x_min', [-3 -5], 'x_max', [3 5]);

            ctrl_x = controllers.ITSMC(dummy_s, dummy_r, est_x);
            ctrl_x.params.dt = dt;
            ctrl_y = controllers.ITSMC(dummy_s, dummy_r, est_y);
            ctrl_y.params.dt = dt;
            ctrl_z = controllers.ITSMC(dummy_s, dummy_r, est_z);
            ctrl_z.params.c1 = 4; ctrl_z.params.c2 = 2;
            ctrl_z.params.K = 1.0; ctrl_z.params.dt = dt;

            arch = architectures.CascadedSMC(ctrl_x, ctrl_y, ctrl_z);

            ref_fn  = utils.references.circular_3d(2, 0.5, 1, 0.3, 0.2);
            dist_fn = utils.disturbances.none(12);
            sim     = benchmark.Simulator('dt', dt, 'T', 5);

            result = sim.run(arch, quad, ref_fn, dist_fn);

            testCase.verifyTrue(all(isfinite(result.x(:))), ...
                'Quadrotor ITSMC produced NaN/Inf');
            testCase.verifyEqual(size(result.x, 1), 12);
            testCase.verifyEqual(size(result.u, 1), 4);
        end

        %% === Nanopositioner Pipeline ===
        function test_nanopositioner_nftsmc(testCase)
            nano = plants.DualStageNanopositioner();
            dt   = 1e-5;

            surf  = surfaces.NonsingularTerminalSurface('beta', 120, 'p', 7, 'q', 5);
            reach = reaching.PowerRate('k', 80, 'alpha', 5/7);
            ctrl  = controllers.NFTSMC(surf, reach);

            g = nano.get_input_gain();
            p = nano.params;
            ff_corr = abs(g) / (p.Kp1 + p.Kp2) - p.wn1^2;

            ctrl.params.input_gain     = g;
            ctrl.params.use_feedforward = true;
            ctrl.params.wn             = p.wn1;
            ctrl.params.zeta           = p.z1;
            ctrl.params.ff_corr        = ff_corr;
            ctrl.params.kI             = 5e8;
            ctrl.params.dt             = dt;

            arch = architectures.DirectSMC(ctrl);

            ref_fn  = @(t) [1e-6 * (t >= 1e-3); 0; 0; 0];
            dist_fn = @(t) zeros(4, 1);
            sim     = benchmark.Simulator('dt', dt, 'T', 5e-3);

            result = sim.run(arch, nano, ref_fn, dist_fn);

            testCase.verifyTrue(all(isfinite(result.x(:))), ...
                'Nanopositioner NFTSMC produced NaN/Inf');
            testCase.verifyEqual(size(result.x, 1), 4);
            testCase.verifyEqual(size(result.u, 1), 1);
        end

        function test_nanopositioner_classical(testCase)
            nano = plants.DualStageNanopositioner();
            dt   = 1e-5;

            surf  = surfaces.LinearSurface('c', 500);
            reach = reaching.SuperTwisting('k1', 300, 'k2', 150);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = @(t) [1e-6 * (t >= 1e-3); 0; 0; 0];
            dist_fn = @(t) zeros(4, 1);
            sim     = benchmark.Simulator('dt', dt, 'T', 5e-3);

            result = sim.run(arch, nano, ref_fn, dist_fn);
            testCase.verifyTrue(all(isfinite(result.x(:))), ...
                'Nanopositioner ClassicalSMC produced NaN/Inf');
        end

        %% === Architecture Reset ===
        function test_architecture_reset_clears_state(testCase)
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 20);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);
            plant = plants.DoubleIntegrator();

            % Run a compute step to populate state
            ctrl.compute(0, [1;0], [0;0], plant);
            testCase.verifyNotEmpty(ctrl.state);

            % Reset and verify
            arch.reset();
            testCase.verifyEqual(ctrl.state.eint, 0);
        end

        %% === Describe Chain ===
        function test_full_describe_chain(testCase)
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 20);
            est   = estimators.NoEstimator();
            ctrl  = controllers.ClassicalSMC(surf, reach, est);
            arch  = architectures.DirectSMC(ctrl);

            info = arch.describe();
            testCase.verifyEqual(info.name, 'Direct');
            testCase.verifyTrue(iscell(info.controllers));
            testCase.verifyEqual(info.controllers{1}.name, 'ClassicalSMC');
            testCase.verifyEqual(info.controllers{1}.surface.name, 'Linear');
            testCase.verifyEqual(info.controllers{1}.reaching.name, 'ConstantRate');
        end
    end
end
