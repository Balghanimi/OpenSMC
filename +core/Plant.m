classdef (Abstract) Plant < handle
    %PLANT Abstract base class for dynamical system models.
    %
    %   Defines the standard interface for any plant used in the benchmark.
    %   Each plant provides its dynamics (ODE), output equation, and metadata.
    %
    %   Plants can be fully actuated (robot arm) or underactuated (quadrotor,
    %   crane, pendulum), which determines which controller architecture to use.

    properties (Abstract)
        name        % (string) e.g. 'DoubleIntegrator', 'Quadrotor6DOF'
        n_states    % (int) dimension of state vector x
        n_inputs    % (int) dimension of control input u
        n_outputs   % (int) dimension of measured output y
        n_dof       % (int) degrees of freedom
        is_underactuated  % (logical) true if n_inputs < n_dof
    end

    properties
        params  % (struct) Physical parameters (mass, inertia, lengths, ...)
        x0      % (n_states x 1) Default initial condition
    end

    methods (Abstract)
        %DYNAMICS State derivative function.
        %   xdot = dynamics(obj, t, x, u, d)
        %
        %   Inputs:
        %       t - (scalar) time
        %       x - (n_states x 1) state vector
        %       u - (n_inputs x 1) control input
        %       d - (n_states x 1) external disturbance (additive)
        %
        %   Output:
        %       xdot - (n_states x 1) state derivative
        xdot = dynamics(obj, t, x, u, d)

        %OUTPUT Output equation.
        %   y = output(obj, x)
        y = output(obj, x)

        %GET_ERROR Compute tracking error from state and reference.
        %   [e, edot] = get_error(obj, x, xref)
        %
        %   This is plant-specific because different plants define
        %   "error" differently (position error, angle wrapping, etc.)
        [e, edot] = get_error(obj, x, xref)
    end

    methods
        function info = describe(obj)
            info.name             = obj.name;
            info.n_states         = obj.n_states;
            info.n_inputs         = obj.n_inputs;
            info.n_outputs        = obj.n_outputs;
            info.n_dof            = obj.n_dof;
            info.is_underactuated = obj.is_underactuated;
            info.params           = obj.params;
        end
    end
end
