classdef (Abstract) Estimator < handle
    %ESTIMATOR Abstract base class for disturbance/uncertainty estimators.
    %
    %   Estimators provide online estimates of lumped disturbances,
    %   unmodeled dynamics, or parameter uncertainties. They decouple
    %   the robustness mechanism from the sliding surface design.
    %
    %   Types: None (use switching gain), DOB, ESO, RBF-ELM, Neural Net

    properties (Abstract)
        name    % (string) e.g. 'None', 'DOB', 'ESO', 'RBF_ELM'
    end

    properties
        params  % (struct) Estimator parameters
        state   % (struct) Internal state (filter states, weights, etc.)
    end

    methods (Abstract)
        %ESTIMATE Compute disturbance estimate.
        %   dhat = estimate(obj, t, x, u, y)
        %
        %   Inputs:
        %       t - (scalar) time
        %       x - (n x 1) state vector (or estimate)
        %       u - (m x 1) control input
        %       y - (p x 1) measured output
        %
        %   Output:
        %       dhat - (n x 1) estimated disturbance
        dhat = estimate(obj, t, x, u, y)

        %UPDATE Online weight/parameter update step.
        %   update(obj, t, x, u, y, dt)
        update(obj, t, x, u, y, dt)
    end

    methods
        function reset(obj)
            obj.state = struct();
        end

        function info = describe(obj)
            info.name   = obj.name;
            info.params = obj.params;
            info.class  = class(obj);
        end
    end
end
