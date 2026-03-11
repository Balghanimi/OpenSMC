classdef ConstantRate < core.ReachingLaw
    %CONSTANTRATE Classic sign-based reaching law.
    %
    %   u_r = -k * sign(s)
    %
    %   Simplest reaching law. Guarantees reaching in finite time.
    %   Produces chattering due to discontinuous sign function.

    properties
        name = 'ConstantRate'
    end

    methods
        function obj = ConstantRate(varargin)
            p.k = 10;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function u_r = compute(obj, s, ~)
            u_r = -obj.params.k * sign(s);
        end
    end
end
