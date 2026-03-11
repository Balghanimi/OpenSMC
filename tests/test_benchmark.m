classdef test_benchmark < matlab.unittest.TestCase
    %TEST_BENCHMARK Integration tests for the benchmark framework.

    methods (Test)

        %% === Simulator ===
        function test_simulator_rk4(testCase)
            sim = benchmark.Simulator('dt', 1e-3, 'T', 1);

            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 20);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);

            result = sim.run(arch, plant, ref_fn, dist_fn);

            % Check result structure
            testCase.verifyTrue(isfield(result, 't'));
            testCase.verifyTrue(isfield(result, 'x'));
            testCase.verifyTrue(isfield(result, 'u'));
            testCase.verifyTrue(isfield(result, 's'));
            testCase.verifyTrue(isfield(result, 'e'));

            % Check dimensions
            N = numel(result.t);
            testCase.verifyEqual(size(result.x), [2, N]);
            testCase.verifyEqual(size(result.u), [1, N]);

            % Check sliding variable is logged
            testCase.verifyFalse(isempty(result.s));
        end

        function test_simulator_euler(testCase)
            sim = benchmark.Simulator('dt', 1e-3, 'T', 0.5, 'solver', 'euler');

            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 5);
            reach = reaching.Saturation('k', 10, 'phi', 0.1);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);

            result = sim.run(arch, plant, ref_fn, dist_fn);
            testCase.verifyTrue(numel(result.t) > 0);
        end

        function test_simulator_tracking(testCase)
            sim = benchmark.Simulator('dt', 1e-4, 'T', 2);

            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 30);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);

            result = sim.run(arch, plant, ref_fn, dist_fn);

            % After 2 seconds, should track reference reasonably
            final_pos = result.x(1, end);
            testCase.verifyEqual(final_pos, 1, 'AbsTol', 0.1);
        end

        %% === Metrics ===
        function test_metrics_compute_all(testCase)
            % Create a simple result struct
            sim = benchmark.Simulator('dt', 1e-3, 'T', 1);

            plant = plants.DoubleIntegrator();
            surf  = surfaces.LinearSurface('c', 10);
            reach = reaching.ConstantRate('k', 20);
            ctrl  = controllers.ClassicalSMC(surf, reach);
            arch  = architectures.DirectSMC(ctrl);

            ref_fn  = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);

            result = sim.run(arch, plant, ref_fn, dist_fn);
            m = benchmark.Metrics.compute_all(result);

            % All 10 metrics should exist
            testCase.verifyTrue(isfield(m, 'rmse'));
            testCase.verifyTrue(isfield(m, 'mae'));
            testCase.verifyTrue(isfield(m, 'ise'));
            testCase.verifyTrue(isfield(m, 'iae'));
            testCase.verifyTrue(isfield(m, 'settling_time'));
            testCase.verifyTrue(isfield(m, 'overshoot'));
            testCase.verifyTrue(isfield(m, 'control_effort'));
            testCase.verifyTrue(isfield(m, 'chattering_idx'));
            testCase.verifyTrue(isfield(m, 'reaching_time'));
            testCase.verifyTrue(isfield(m, 'steady_state_error'));

            % All should be finite
            testCase.verifyTrue(isfinite(m.rmse));
            testCase.verifyTrue(isfinite(m.mae));
            testCase.verifyTrue(isfinite(m.ise));
            testCase.verifyTrue(isfinite(m.iae));
            testCase.verifyTrue(isfinite(m.control_effort));
            testCase.verifyTrue(isfinite(m.chattering_idx));
        end

        function test_metrics_underactuated(testCase)
            % Test max_swing and residual_swing
            result = struct();
            result.x = [0 0 0; 0 0 0; 0.1 0.2 0.05; 0 0 0];
            result.t = [0, 0.5, 1.0];

            ms = benchmark.Metrics.max_swing(result, 3);
            testCase.verifyEqual(ms, 0.2, 'AbsTol', 1e-10);

            rs = benchmark.Metrics.residual_swing(result, 3);
            testCase.verifyTrue(rs >= 0);
        end

        %% === BenchmarkRunner ===
        function test_runner_end_to_end(testCase)
            runner = benchmark.BenchmarkRunner('dt', 1e-3, 'T', 1);

            % Two architectures
            surf1  = surfaces.LinearSurface('c', 10);
            reach1 = reaching.ConstantRate('k', 20);
            ctrl1  = controllers.ClassicalSMC(surf1, reach1);
            arch1  = architectures.DirectSMC(ctrl1);

            surf2  = surfaces.TerminalSurface('beta', 10, 'p', 5, 'q', 7);
            reach2 = reaching.Saturation('k', 20, 'phi', 0.1);
            ctrl2  = controllers.ClassicalSMC(surf2, reach2);
            arch2  = architectures.DirectSMC(ctrl2);

            runner.add_architecture('ClassicalSMC',  arch1);
            runner.add_architecture('TerminalSMC',   arch2);

            % One plant
            plant  = plants.DoubleIntegrator();
            ref_fn = utils.references.step_ref(1, 2);
            dist_fn = utils.disturbances.none(2);
            runner.add_plant('DoubleIntegrator', plant, ref_fn, dist_fn);

            results = runner.run_all();

            testCase.verifyEqual(size(results), [2, 1]);
            testCase.verifyNotEmpty(results(1,1).metrics);
            testCase.verifyNotEmpty(results(2,1).metrics);
        end

        %% === Reference Generators ===
        function test_ref_step(testCase)
            fn = utils.references.step_ref(2, 4);
            xref = fn(5);
            testCase.verifyEqual(xref, [2; 0; 0; 0]);
        end

        function test_ref_ramp(testCase)
            fn = utils.references.ramp(0.5, 2);
            xref = fn(4);
            testCase.verifyEqual(xref, [2; 0.5]);
        end

        function test_ref_sinusoidal(testCase)
            fn = utils.references.sinusoidal(1, 1, 2);
            xref = fn(0);
            testCase.verifyEqual(xref(1), 0, 'AbsTol', 1e-10);
        end

        function test_ref_transport(testCase)
            fn = utils.references.transport(2.0, 4);
            xref = fn(0);
            testCase.verifyEqual(xref, [2; 0; 0; 0]);
        end

        function test_ref_circular_3d(testCase)
            fn = utils.references.circular_3d(2, 0.5, 1, 0.3, 0.2);
            xref = fn(0);
            testCase.verifyEqual(numel(xref), 12);
        end

        function test_ref_multi_step(testCase)
            fn = utils.references.multi_step([1 2 3], [0 2 4], 2);
            xref = fn(3);
            testCase.verifyEqual(xref, [2; 0]);
        end

        %% === Disturbance Generators ===
        function test_dist_none(testCase)
            fn = utils.disturbances.none(3);
            d = fn(5);
            testCase.verifyEqual(d, zeros(3,1));
        end

        function test_dist_step(testCase)
            fn = utils.disturbances.step(2, 1, 2);
            d_before = fn(0.5);
            d_after  = fn(1.5);
            testCase.verifyEqual(d_before, [0; 0]);
            testCase.verifyEqual(d_after, [2; 2]);
        end

        function test_dist_sinusoidal(testCase)
            fn = utils.disturbances.sinusoidal(1, 1, 2);
            d = fn(0.25);
            testCase.verifyTrue(all(isfinite(d)));
        end

        function test_dist_composite(testCase)
            fn = utils.disturbances.composite({
                utils.disturbances.step(1, 0, 2), ...
                utils.disturbances.step(2, 0, 2)
            });
            d = fn(1);
            testCase.verifyEqual(d, [3; 3]);
        end

        function test_dist_random_band(testCase)
            fn = utils.disturbances.random_band(1, 42, 2);
            d = fn(0.5);
            testCase.verifyEqual(numel(d), 2);
            testCase.verifyTrue(all(isfinite(d)));
        end
    end
end
