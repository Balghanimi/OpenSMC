classdef ExponentialRate < core.ReachingLaw
    %EXPONENTIALRATE Exponential reaching law.
    %
    %   u_r = -k * sign(s) - q * s
    %
    %   Adds a linear term for faster reaching when far from the surface.
    %   The exponential decay of |s| reduces chattering near s = 0.
    %
    %   Reference: Gao, W., & Hung, J.C. (1993). "Variable structure
    %   control of nonlinear systems: A new approach." IEEE Trans.
    %   Industrial Electronics, 40(1), 45-55.

    properties
        name = 'ExponentialRate'
    end

    methods
        function obj = ExponentialRate(varargin)
            p.k = 10;    % sign gain
            p.q = 5;     % linear gain
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function u_r = compute(obj, s, ~)
            u_r = -obj.params.k * sign(s) - obj.params.q * s;
        end
    end
end
