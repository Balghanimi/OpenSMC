classdef test_surfaces < matlab.unittest.TestCase
    %TEST_SURFACES Unit tests for all sliding surface implementations.

    methods (Test)

        %% === LinearSurface ===
        function test_linear_basic(testCase)
            s = surfaces.LinearSurface('c', 5);
            testCase.verifyEqual(s.compute(1, 2, 0, 0), 7);   % 2 + 5*1
            testCase.verifyEqual(s.compute(0, 0, 0, 0), 0);   % origin
        end

        function test_linear_negative_error(testCase)
            s = surfaces.LinearSurface('c', 10);
            testCase.verifyEqual(s.compute(-1, 3, 0, 0), -7); % 3 + 10*(-1)
        end

        function test_linear_vector(testCase)
            s = surfaces.LinearSurface('c', 2);
            e = [1; -1]; edot = [0.5; 0.5];
            result = s.compute(e, edot, zeros(2,1), 0);
            testCase.verifyEqual(result, [2.5; -1.5]);
        end

        function test_linear_describe(testCase)
            s = surfaces.LinearSurface('c', 7);
            info = s.describe();
            testCase.verifyEqual(info.name, 'Linear');
            testCase.verifyEqual(info.params.c, 7);
        end

        %% === TerminalSurface ===
        function test_terminal_basic(testCase)
            s = surfaces.TerminalSurface('beta', 1, 'p', 5, 'q', 7);
            result = s.compute(1, 0, 0, 0);
            % s = 0 + 1*|1|^(5/7)*sign(1) = 1
            testCase.verifyEqual(result, 1, 'AbsTol', 1e-10);
        end

        function test_terminal_negative(testCase)
            s = surfaces.TerminalSurface('beta', 2, 'p', 5, 'q', 7);
            result = s.compute(-1, 0, 0, 0);
            % s = 0 + 2*|-1|^(5/7)*(-1) = -2
            testCase.verifyEqual(result, -2, 'AbsTol', 1e-10);
        end

        function test_terminal_zero_error(testCase)
            s = surfaces.TerminalSurface('beta', 10, 'p', 5, 'q', 7);
            result = s.compute(0, 5, 0, 0);
            % s = 5 + 10*0 = 5
            testCase.verifyEqual(result, 5, 'AbsTol', 1e-10);
        end

        %% === NonsingularTerminalSurface ===
        function test_nst_basic(testCase)
            s = surfaces.NonsingularTerminalSurface('beta', 1, 'p', 7, 'q', 5);
            result = s.compute(1, 1, 0, 0);
            % s = 1 + (1/1)*|1|^(5/7)*sign(1) = 1 + 1 = 2
            testCase.verifyEqual(result, 2, 'AbsTol', 1e-10);
        end

        function test_nst_fast_version(testCase)
            s = surfaces.NonsingularTerminalSurface('alpha', 0.5, 'beta', 1, 'p', 7, 'q', 5);
            result = s.compute(1, 1, 0, 0);
            % s = 1 + 1*|1|^(5/7) + 0.5*1 = 2.5
            testCase.verifyEqual(result, 2.5, 'AbsTol', 1e-10);
        end

        function test_nst_zero_velocity(testCase)
            s = surfaces.NonsingularTerminalSurface('beta', 10, 'p', 7, 'q', 5);
            result = s.compute(3, 0, 0, 0);
            % s = 3 + (1/10)*|0|^(5/7)*sign(0) = 3
            testCase.verifyEqual(result, 3, 'AbsTol', 1e-10);
        end

        %% === FastTerminalSurface ===
        function test_fast_terminal(testCase)
            s = surfaces.FastTerminalSurface('alpha', 2, 'beta', 1, 'p', 9, 'q', 5);
            result = s.compute(1, 1, 0, 0);
            % s = 1 + 2*1 + 1*|1|^(5/9)*1 = 1 + 2 + 1 = 4
            testCase.verifyEqual(result, 4, 'AbsTol', 1e-10);
        end

        function test_fast_terminal_origin(testCase)
            s = surfaces.FastTerminalSurface();
            testCase.verifyEqual(s.compute(0, 0, 0, 0), 0, 'AbsTol', 1e-10);
        end

        %% === IntegralTerminalSurface ===
        function test_integral_terminal(testCase)
            s = surfaces.IntegralTerminalSurface('c1', 3, 'c2', 1, 'p', 5, 'q', 7);
            result = s.compute(1, 2, 0.5, 0);
            % eint = 0.5 => frac_int = |0.5|^(5/7)*sign(0.5)
            pq = 5/7;
            frac_int = abs(0.5)^pq * 1;
            expected = 2 + 3*1 + 1*frac_int;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end

        function test_integral_terminal_origin(testCase)
            s = surfaces.IntegralTerminalSurface('c1', 10, 'c2', 5, 'p', 5, 'q', 7);
            testCase.verifyEqual(s.compute(0, 0, 0, 0), 0, 'AbsTol', 1e-10);
        end

        %% === HierarchicalSurface ===
        function test_hierarchical(testCase)
            s = surfaces.HierarchicalSurface('c1', 5, 'c2', 3, 'lambda', 2, ...
                'idx_a', 1, 'idx_u', 2, 'idx_adot', 1, 'idx_udot', 2);
            e = [1; 0.5]; edot = [2; 1];
            result = s.compute(e, edot, zeros(2,1), 0);
            % s1 = 2 + 5*1 = 7,  s2 = 1 + 3*0.5 = 2.5
            % S = 7 + 2*2.5 = 12
            testCase.verifyEqual(result, 12, 'AbsTol', 1e-10);
        end

        %% === IntegralSlidingSurface ===
        function test_integral_sliding(testCase)
            s = surfaces.IntegralSlidingSurface();
            testCase.verifyClass(s, 'surfaces.IntegralSlidingSurface');
            testCase.verifyEqual(s.name, 'IntegralSliding');
        end

        %% === PIDSurface ===
        function test_pid_surface(testCase)
            s = surfaces.PIDSurface('alpha', 2, 'beta', 1, 'gamma', 0.5);
            result = s.compute(1, 3, 2, 0);
            % s = 2*3 + 1*1 + 0.5*2 = 6 + 1 + 1 = 8
            testCase.verifyEqual(result, 8, 'AbsTol', 1e-10);
        end

        %% === NonlinearDampingSurface ===
        function test_nonlinear_damping(testCase)
            s = surfaces.NonlinearDampingSurface();
            testCase.verifyClass(s, 'surfaces.NonlinearDampingSurface');
            testCase.verifyEqual(s.name, 'NonlinearDamping');
        end

        %% === Common Properties ===
        function test_all_surfaces_have_name(testCase)
            surfs = {
                surfaces.LinearSurface(), ...
                surfaces.TerminalSurface(), ...
                surfaces.NonsingularTerminalSurface(), ...
                surfaces.FastTerminalSurface(), ...
                surfaces.IntegralTerminalSurface(), ...
                surfaces.HierarchicalSurface(), ...
                surfaces.IntegralSlidingSurface(), ...
                surfaces.PIDSurface(), ...
                surfaces.NonlinearDampingSurface()
            };
            for i = 1:numel(surfs)
                testCase.verifyNotEmpty(surfs{i}.name);
                testCase.verifyNotEmpty(surfs{i}.params);
            end
        end

        function test_all_surfaces_reset(testCase)
            surfs = {
                surfaces.LinearSurface(), ...
                surfaces.TerminalSurface(), ...
                surfaces.NonsingularTerminalSurface(), ...
                surfaces.FastTerminalSurface(), ...
                surfaces.IntegralTerminalSurface(), ...
                surfaces.PIDSurface()
            };
            for i = 1:numel(surfs)
                surfs{i}.reset();  % should not error
            end
        end
    end
end
