classdef test_plants < matlab.unittest.TestCase
    %TEST_PLANTS Unit tests for all plant implementations.

    methods (Test)

        %% === DoubleIntegrator ===
        function test_di_dimensions(testCase)
            p = plants.DoubleIntegrator();
            testCase.verifyEqual(p.n_states, 2);
            testCase.verifyEqual(p.n_inputs, 1);
            testCase.verifyEqual(p.is_underactuated, false);
        end

        function test_di_dynamics_free(testCase)
            p = plants.DoubleIntegrator();
            xdot = p.dynamics(0, [0; 1], 0, [0; 0]);
            % x1dot = x2 = 1,  x2dot = u + d = 0
            testCase.verifyEqual(xdot, [1; 0]);
        end

        function test_di_dynamics_forced(testCase)
            p = plants.DoubleIntegrator();
            xdot = p.dynamics(0, [0; 0], 5, [0; 0]);
            testCase.verifyEqual(xdot, [0; 5]);
        end

        function test_di_dynamics_disturbed(testCase)
            p = plants.DoubleIntegrator();
            xdot = p.dynamics(0, [0; 0], 0, [0; 3]);
            testCase.verifyEqual(xdot, [0; 3]);
        end

        function test_di_output(testCase)
            p = plants.DoubleIntegrator();
            testCase.verifyEqual(p.output([5; 3]), 5);
        end

        function test_di_get_error(testCase)
            p = plants.DoubleIntegrator();
            [e, edot] = p.get_error([1; 2], [3; 4]);
            testCase.verifyEqual(e, 2);
            testCase.verifyEqual(edot, 2);
        end

        function test_di_initial_condition(testCase)
            p = plants.DoubleIntegrator('x0', [1; -1]);
            testCase.verifyEqual(p.x0, [1; -1]);
        end

        %% === InvertedPendulum ===
        function test_ip_dimensions(testCase)
            p = plants.InvertedPendulum();
            testCase.verifyEqual(p.n_states, 4);
            testCase.verifyEqual(p.n_inputs, 1);
            testCase.verifyEqual(p.is_underactuated, true);
        end

        function test_ip_equilibrium(testCase)
            p = plants.InvertedPendulum();
            xdot = p.dynamics(0, [0; 0; 0; 0], 0, zeros(4,1));
            % At origin (upright), xdot should be all zeros
            testCase.verifyEqual(xdot, zeros(4,1), 'AbsTol', 1e-10);
        end

        function test_ip_output(testCase)
            p = plants.InvertedPendulum();
            y = p.output([1; 2; 0.5; 3]);
            testCase.verifyEqual(y, [1; 0.5]);
        end

        %% === SinglePendulumCrane ===
        function test_crane_dimensions(testCase)
            p = plants.SinglePendulumCrane();
            testCase.verifyEqual(p.n_states, 4);
            testCase.verifyEqual(p.n_inputs, 1);
            testCase.verifyEqual(p.is_underactuated, true);
        end

        function test_crane_equilibrium(testCase)
            p = plants.SinglePendulumCrane();
            xdot = p.dynamics(0, [0; 0; 0; 0], 0, zeros(4,1));
            testCase.verifyEqual(xdot, zeros(4,1), 'AbsTol', 1e-10);
        end

        function test_crane_dynamics_components(testCase)
            p = plants.SinglePendulumCrane();
            [f1, f2, b1, b2] = p.get_dynamics_components([0; 0; 0; 0]);
            % At equilibrium, f1=0, f2=0
            testCase.verifyEqual(f1, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(f2, 0, 'AbsTol', 1e-10);
            % b1, b2 should be nonzero (controllable)
            testCase.verifyNotEqual(b1, 0);
            testCase.verifyNotEqual(b2, 0);
        end

        function test_crane_output(testCase)
            p = plants.SinglePendulumCrane();
            y = p.output([2; 1; 0.1; 0.3]);
            testCase.verifyEqual(y, [2; 0.1]);
        end

        function test_crane_get_error(testCase)
            p = plants.SinglePendulumCrane();
            [e, edot] = p.get_error([1; 2; 0.1; 0.3], [3; 0; 0; 0]);
            testCase.verifyEqual(e, [2; -0.1], 'AbsTol', 1e-10);
            testCase.verifyEqual(edot, [-2; -0.3], 'AbsTol', 1e-10);
        end

        %% === DoublePendulumCrane ===
        function test_double_crane_dimensions(testCase)
            p = plants.DoublePendulumCrane();
            testCase.verifyEqual(p.n_states, 6);
            testCase.verifyEqual(p.n_inputs, 1);
            testCase.verifyEqual(p.is_underactuated, true);
        end

        function test_double_crane_equilibrium(testCase)
            p = plants.DoublePendulumCrane();
            xdot = p.dynamics(0, zeros(6,1), 0, zeros(6,1));
            testCase.verifyEqual(xdot, zeros(6,1), 'AbsTol', 1e-10);
        end

        %% === Quadrotor6DOF ===
        function test_quad_dimensions(testCase)
            p = plants.Quadrotor6DOF();
            testCase.verifyEqual(p.n_states, 12);
            testCase.verifyEqual(p.n_inputs, 4);
            testCase.verifyEqual(p.is_underactuated, true);
        end

        function test_quad_hover(testCase)
            p = plants.Quadrotor6DOF();
            % Hover thrust: U1 = m*g
            u_hover = [p.params.m * p.params.g; 0; 0; 0];
            xdot = p.dynamics(0, zeros(12,1), u_hover, zeros(12,1));
            % At hover, all derivatives should be ~0
            testCase.verifyEqual(xdot, zeros(12,1), 'AbsTol', 1e-10);
        end

        function test_quad_freefall(testCase)
            p = plants.Quadrotor6DOF();
            xdot = p.dynamics(0, zeros(12,1), [0;0;0;0], zeros(12,1));
            % Only az = -g should be nonzero
            testCase.verifyEqual(xdot(9), -p.params.g, 'AbsTol', 1e-10);
            testCase.verifyEqual(xdot(1:8), zeros(8,1), 'AbsTol', 1e-10);
        end

        function test_quad_output(testCase)
            p = plants.Quadrotor6DOF();
            x = (1:12)';
            y = p.output(x);
            testCase.verifyEqual(y, (1:6)');
        end

        function test_quad_get_error(testCase)
            p = plants.Quadrotor6DOF();
            x    = zeros(12,1);
            xref = ones(12,1);
            [e, edot] = p.get_error(x, xref);
            testCase.verifyEqual(e, ones(6,1));
            testCase.verifyEqual(edot, ones(6,1));
        end

        function test_quad_params(testCase)
            p = plants.Quadrotor6DOF();
            testCase.verifyEqual(p.params.m, 0.468, 'AbsTol', 1e-10);
            testCase.verifyEqual(p.params.g, 9.81, 'AbsTol', 1e-10);
        end

        %% === DualStageNanopositioner ===
        function test_nano_dimensions(testCase)
            p = plants.DualStageNanopositioner();
            testCase.verifyEqual(p.n_states, 4);
            testCase.verifyEqual(p.n_inputs, 1);
            testCase.verifyEqual(p.n_outputs, 1);
            testCase.verifyEqual(p.n_dof, 1);
            testCase.verifyEqual(p.is_underactuated, false);
        end

        function test_nano_equilibrium(testCase)
            p = plants.DualStageNanopositioner();
            xdot = p.dynamics(0, zeros(4,1), 0, zeros(4,1));
            testCase.verifyEqual(xdot, zeros(4,1), 'AbsTol', 1e-10);
        end

        function test_nano_dynamics_forced(testCase)
            p = plants.DualStageNanopositioner();
            xdot = p.dynamics(0, zeros(4,1), 1.0, zeros(4,1));
            % x1dot = x2 = 0, x2dot = Kp1*wn1^2*u
            testCase.verifyEqual(xdot(1), 0, 'AbsTol', 1e-10);
            expected_x2dot = p.params.Kp1 * p.params.wn1^2;
            testCase.verifyEqual(xdot(2), expected_x2dot, 'RelTol', 1e-10);
            testCase.verifyEqual(xdot(3), 0, 'AbsTol', 1e-10);
            expected_x4dot = p.params.Kp2 * p.params.wn2^2;
            testCase.verifyEqual(xdot(4), expected_x4dot, 'RelTol', 1e-10);
        end

        function test_nano_output(testCase)
            p = plants.DualStageNanopositioner();
            y = p.output([1e-6; 0; 0.5e-6; 0]);
            testCase.verifyEqual(y, 1.5e-6, 'AbsTol', 1e-15);
        end

        function test_nano_get_error(testCase)
            p = plants.DualStageNanopositioner();
            x = [0.5e-6; 1e-6; 0.3e-6; 0.2e-6];
            xref = [1e-6; 0; 0; 0];
            [e, edot] = p.get_error(x, xref);
            % y = 0.5e-6 + 0.3e-6 = 0.8e-6,  ydot = 1e-6 + 0.2e-6 = 1.2e-6
            testCase.verifyEqual(e, 1e-6 - 0.8e-6, 'AbsTol', 1e-15);
            testCase.verifyEqual(edot, 0 - 1.2e-6, 'AbsTol', 1e-15);
        end

        function test_nano_input_gain(testCase)
            p = plants.DualStageNanopositioner();
            g = p.get_input_gain();
            % g should be negative (more voltage -> more displacement -> less error)
            testCase.verifyLessThan(g, 0);
            expected = -(p.params.Kp1 * p.params.wn1^2 + p.params.Kp2 * p.params.wn2^2);
            testCase.verifyEqual(g, expected, 'RelTol', 1e-10);
        end

        function test_nano_ss_matrices(testCase)
            p = plants.DualStageNanopositioner();
            [A, B, C] = p.get_ss_matrices();
            testCase.verifySize(A, [4, 4]);
            testCase.verifySize(B, [4, 1]);
            testCase.verifySize(C, [1, 4]);
            % Verify equilibrium: A*0 + B*0 = 0
            testCase.verifyEqual(A * zeros(4,1) + B * 0, zeros(4,1));
        end

        function test_nano_voltage_clamp(testCase)
            p = plants.DualStageNanopositioner();
            % 100V should be clamped to 20V
            xdot1 = p.dynamics(0, zeros(4,1), 100, zeros(4,1));
            xdot2 = p.dynamics(0, zeros(4,1), 20, zeros(4,1));
            testCase.verifyEqual(xdot1, xdot2, 'AbsTol', 1e-10);
        end

        %% === TwoLinkArm ===
        function test_arm_dimensions(testCase)
            p = plants.TwoLinkArm();
            testCase.verifyEqual(p.n_states, 4);
            testCase.verifyEqual(p.n_inputs, 2);
            testCase.verifyEqual(p.n_dof, 2);
            testCase.verifyEqual(p.is_underactuated, false);
        end

        function test_arm_equilibrium(testCase)
            p = plants.TwoLinkArm();
            % At q=[0;0], gravity torques are nonzero, so we need
            % compensating torques for equilibrium
            [~, ~, G] = p.get_dynamics_matrices([0; 0; 0; 0]);
            xdot = p.dynamics(0, [0; 0; 0; 0], G, zeros(4,1));
            testCase.verifyEqual(xdot, zeros(4,1), 'AbsTol', 1e-10);
        end

        function test_arm_freefall(testCase)
            p = plants.TwoLinkArm();
            xdot = p.dynamics(0, [0; 0; 0; 0], [0; 0], zeros(4,1));
            % With gravity and zero torque, joint accelerations should be nonzero
            testCase.verifyNotEqual(xdot(2), 0);
        end

        function test_arm_output(testCase)
            p = plants.TwoLinkArm();
            y = p.output([0.5; 1; 0.3; 2]);
            testCase.verifyEqual(y, [0.5; 0.3]);
        end

        function test_arm_get_error(testCase)
            p = plants.TwoLinkArm();
            [e, edot] = p.get_error([0.5; 1; 0.3; 2], [1; 0; 1; 0]);
            testCase.verifyEqual(e, [0.5; 0.7], 'AbsTol', 1e-10);
            testCase.verifyEqual(edot, [-1; -2], 'AbsTol', 1e-10);
        end

        function test_arm_dynamics_matrices(testCase)
            p = plants.TwoLinkArm();
            [M, C, G] = p.get_dynamics_matrices([0; 0; 0; 0]);
            testCase.verifySize(M, [2, 2]);
            testCase.verifySize(C, [2, 1]);
            testCase.verifySize(G, [2, 1]);
            % M should be symmetric positive definite
            testCase.verifyEqual(M(1,2), M(2,1), 'AbsTol', 1e-10);
            testCase.verifyGreaterThan(det(M), 0);
        end

        function test_arm_custom_params(testCase)
            p = plants.TwoLinkArm('m1', 2.0, 'm2', 1.5);
            testCase.verifyEqual(p.params.m1, 2.0);
            testCase.verifyEqual(p.params.m2, 1.5);
        end

        %% === PMSM ===
        function test_pmsm_dimensions(testCase)
            p = plants.PMSM();
            testCase.verifyEqual(p.n_states, 4);
            testCase.verifyEqual(p.n_inputs, 2);
            testCase.verifyEqual(p.n_dof, 1);
            testCase.verifyEqual(p.is_underactuated, false);
        end

        function test_pmsm_equilibrium(testCase)
            p = plants.PMSM();
            xdot = p.dynamics(0, zeros(4,1), [0; 0], zeros(4,1));
            testCase.verifyEqual(xdot, zeros(4,1), 'AbsTol', 1e-10);
        end

        function test_pmsm_output(testCase)
            p = plants.PMSM();
            y = p.output([0.1; 0.2; 100; 3.14]);
            testCase.verifyEqual(y, [100; 3.14]);
        end

        function test_pmsm_torque(testCase)
            p = plants.PMSM();
            % For SPMSM (Ld=Lq), Te = 1.5*pp*psi_f*iq
            x = [0; 1; 0; 0];  % iq = 1
            Te = p.get_torque(x);
            expected = 1.5 * p.params.pp * p.params.psi_f * 1;
            testCase.verifyEqual(Te, expected, 'AbsTol', 1e-10);
        end

        function test_pmsm_voltage_clamp(testCase)
            p = plants.PMSM();
            xdot1 = p.dynamics(0, zeros(4,1), [100; 100], zeros(4,1));
            xdot2 = p.dynamics(0, zeros(4,1), [48; 48], zeros(4,1));
            testCase.verifyEqual(xdot1, xdot2, 'AbsTol', 1e-10);
        end

        %% === SurfaceVessel ===
        function test_vessel_dimensions(testCase)
            p = plants.SurfaceVessel();
            testCase.verifyEqual(p.n_states, 6);
            testCase.verifyEqual(p.n_inputs, 3);
            testCase.verifyEqual(p.n_dof, 3);
        end

        function test_vessel_equilibrium(testCase)
            p = plants.SurfaceVessel();
            xdot = p.dynamics(0, zeros(6,1), [0;0;0], zeros(6,1));
            testCase.verifyEqual(xdot, zeros(6,1), 'AbsTol', 1e-10);
        end

        function test_vessel_output(testCase)
            p = plants.SurfaceVessel();
            y = p.output([10; 20; 0.5; 1; 2; 0.1]);
            testCase.verifyEqual(y, [10; 20; 0.5]);
        end

        function test_vessel_get_error(testCase)
            p = plants.SurfaceVessel();
            x = [1; 2; 0.1; 0.5; 0.3; 0.01];
            xref = [5; 5; 0; 0; 0; 0];
            [e, edot] = p.get_error(x, xref);
            testCase.verifyEqual(e, [4; 3; -0.1], 'AbsTol', 1e-10);
        end

        %% === All Plants ===
        function test_all_plants_describe(testCase)
            ps = {
                plants.DoubleIntegrator(), ...
                plants.InvertedPendulum(), ...
                plants.SinglePendulumCrane(), ...
                plants.DoublePendulumCrane(), ...
                plants.Quadrotor6DOF(), ...
                plants.DualStageNanopositioner(), ...
                plants.TwoLinkArm(), ...
                plants.PMSM(), ...
                plants.SurfaceVessel()
            };
            for i = 1:numel(ps)
                info = ps{i}.describe();
                testCase.verifyTrue(isfield(info, 'name'));
                testCase.verifyTrue(isfield(info, 'class'));
            end
        end

        function test_all_plants_initial_conditions(testCase)
            ps = {
                plants.DoubleIntegrator(), ...
                plants.InvertedPendulum(), ...
                plants.SinglePendulumCrane(), ...
                plants.DoublePendulumCrane(), ...
                plants.Quadrotor6DOF(), ...
                plants.DualStageNanopositioner(), ...
                plants.TwoLinkArm(), ...
                plants.PMSM(), ...
                plants.SurfaceVessel()
            };
            for i = 1:numel(ps)
                testCase.verifyEqual(numel(ps{i}.x0), ps{i}.n_states);
            end
        end
    end
end
