classdef PowerRate < core.ReachingLaw
    %POWERRATE Power-rate reaching law.
    %
    %   u_r = -k * |s|^alpha * sign(s)
    %
    %   alpha < 1: faster reaching near s = 0 (finite-time)
    %   alpha > 1: slower reaching near s = 0 (smoother)
    %   alpha = 1: equivalent to constant rate (scaled by |s|)

    properties
        name = 'PowerRate'
    end

    methods
        function obj = PowerRate(varargin)
            p.k     = 10;
            p.alpha = 0.5;  % power (< 1 for finite-time reaching)
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function u_r = compute(obj, s, ~)
            u_r = -obj.params.k * abs(s).^obj.params.alpha .* sign(s);
        end
    end
end
