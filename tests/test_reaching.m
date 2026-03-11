classdef test_reaching < matlab.unittest.TestCase
    %TEST_REACHING Unit tests for all reaching law implementations.

    methods (Test)

        %% === ConstantRate ===
        function test_constant_positive_s(testCase)
            r = reaching.ConstantRate('k', 5);
            testCase.verifyEqual(r.compute(2, 0), -5);    % -k*sign(2)
        end

        function test_constant_negative_s(testCase)
            r = reaching.ConstantRate('k', 5);
            testCase.verifyEqual(r.compute(-3, 0), 5);    % -k*sign(-3)
        end

        function test_constant_zero_s(testCase)
            r = reaching.ConstantRate('k', 10);
            testCase.verifyEqual(r.compute(0, 0), 0);     % -k*sign(0) = 0
        end

        function test_constant_vector(testCase)
            r = reaching.ConstantRate('k', 3);
            result = r.compute([1; -1; 0], 0);
            testCase.verifyEqual(result, [-3; 3; 0]);
        end

        %% === ExponentialRate ===
        function test_exponential_basic(testCase)
            r = reaching.ExponentialRate('k', 2, 'q', 3);
            result = r.compute(1, 0);
            % -2*sign(1) - 3*1 = -2 - 3 = -5
            testCase.verifyEqual(result, -5, 'AbsTol', 1e-10);
        end

        function test_exponential_negative(testCase)
            r = reaching.ExponentialRate('k', 2, 'q', 3);
            result = r.compute(-1, 0);
            % -2*sign(-1) - 3*(-1) = 2 + 3 = 5
            testCase.verifyEqual(result, 5, 'AbsTol', 1e-10);
        end

        function test_exponential_zero(testCase)
            r = reaching.ExponentialRate('k', 10, 'q', 5);
            testCase.verifyEqual(r.compute(0, 0), 0, 'AbsTol', 1e-10);
        end

        %% === PowerRate ===
        function test_power_half(testCase)
            r = reaching.PowerRate('k', 4, 'alpha', 0.5);
            result = r.compute(4, 0);
            % -4*|4|^0.5*sign(4) = -4*2*1 = -8
            testCase.verifyEqual(result, -8, 'AbsTol', 1e-10);
        end

        function test_power_zero(testCase)
            r = reaching.PowerRate('k', 10, 'alpha', 0.5);
            testCase.verifyEqual(r.compute(0, 0), 0, 'AbsTol', 1e-10);
        end

        function test_power_negative(testCase)
            r = reaching.PowerRate('k', 1, 'alpha', 0.5);
            result = r.compute(-9, 0);
            % -1*|-9|^0.5*sign(-9) = -3*(-1) = 3
            testCase.verifyEqual(result, 3, 'AbsTol', 1e-10);
        end

        %% === SuperTwisting ===
        function test_supertwisting_initial(testCase)
            r = reaching.SuperTwisting('k1', 4, 'k2', 2);
            result = r.compute(1, 0);
            % -4*|1|^0.5*sign(1) + v (=0) = -4
            testCase.verifyEqual(result, -4, 'AbsTol', 1e-10);
        end

        function test_supertwisting_update(testCase)
            r = reaching.SuperTwisting('k1', 4, 'k2', 2);
            r.compute(1, 0);
            r.update_state(1, 0.01);
            % v = 0 - 2*sign(1)*0.01 = -0.02
            testCase.verifyEqual(r.state.v, -0.02, 'AbsTol', 1e-10);
        end

        function test_supertwisting_reset(testCase)
            r = reaching.SuperTwisting('k1', 4, 'k2', 2);
            r.compute(1, 0);
            r.update_state(1, 0.01);
            r.reset();
            testCase.verifyEqual(r.state.v, 0);
        end

        %% === Saturation ===
        function test_saturation_inside_boundary(testCase)
            r = reaching.Saturation('k', 10, 'phi', 1);
            result = r.compute(0.5, 0);
            % sat(0.5/1) = 0.5, u_r = -10*0.5 = -5
            testCase.verifyEqual(result, -5, 'AbsTol', 1e-10);
        end

        function test_saturation_outside_boundary(testCase)
            r = reaching.Saturation('k', 10, 'phi', 0.1);
            result = r.compute(5, 0);
            % sat(5/0.1) = sat(50) = 1, u_r = -10*1 = -10
            testCase.verifyEqual(result, -10, 'AbsTol', 1e-10);
        end

        function test_saturation_negative(testCase)
            r = reaching.Saturation('k', 10, 'phi', 0.1);
            result = r.compute(-5, 0);
            % sat(-50) = -1, u_r = -10*(-1) = 10
            testCase.verifyEqual(result, 10, 'AbsTol', 1e-10);
        end

        function test_saturation_zero(testCase)
            r = reaching.Saturation('k', 10, 'phi', 0.1);
            testCase.verifyEqual(r.compute(0, 0), 0, 'AbsTol', 1e-10);
        end

        %% === Common Properties ===
        function test_all_reaching_have_name(testCase)
            laws = {
                reaching.ConstantRate(), ...
                reaching.ExponentialRate(), ...
                reaching.PowerRate(), ...
                reaching.SuperTwisting(), ...
                reaching.Saturation()
            };
            for i = 1:numel(laws)
                testCase.verifyNotEmpty(laws{i}.name);
                testCase.verifyNotEmpty(laws{i}.params);
            end
        end

        function test_all_reaching_describe(testCase)
            laws = {
                reaching.ConstantRate(), ...
                reaching.ExponentialRate(), ...
                reaching.PowerRate(), ...
                reaching.SuperTwisting(), ...
                reaching.Saturation()
            };
            for i = 1:numel(laws)
                info = laws{i}.describe();
                testCase.verifyTrue(isfield(info, 'name'));
                testCase.verifyTrue(isfield(info, 'params'));
            end
        end
    end
end
