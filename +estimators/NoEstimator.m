classdef NoEstimator < core.Estimator
    %NOESTIMATOR Dummy estimator (pure SMC with switching gain only).
    %
    %   Returns zero disturbance estimate. Use this when the reaching
    %   law gain is sufficient for disturbance rejection.

    properties
        name = 'None'
    end

    methods
        function obj = NoEstimator()
            obj.params = struct();
            obj.state  = struct();
        end

        function dhat = estimate(~, ~, x, ~, ~)
            dhat = zeros(size(x));
        end

        function update(~, ~, ~, ~, ~, ~)
            % Nothing to update
        end
    end
end
