classdef Saturation < core.ReachingLaw
    %SATURATION Boundary layer reaching law (chattering reduction).
    %
    %   u_r = -k * sat(s / phi)
    %
    %   where sat(x) = x if |x| <= 1, sign(x) otherwise.
    %
    %   Replaces the discontinuous sign function with a continuous
    %   saturation function within a boundary layer of width phi.
    %   Trades exact sliding for chattering reduction.
    %
    %   Reference: Slotine, J.J.E. (1984). "Sliding controller design
    %   for non-linear systems." IJOC, 40(2), 421-434.

    properties
        name = 'Saturation'
    end

    methods
        function obj = Saturation(varargin)
            p.k   = 10;    % gain
            p.phi = 0.1;   % boundary layer width
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function u_r = compute(obj, s, ~)
            x = s / obj.params.phi;
            sat_x = min(max(x, -1), 1);  % saturation function
            u_r = -obj.params.k * sat_x;
        end
    end
end
