classdef SuperTwisting < core.ReachingLaw
    %SUPERTWISTING Super-twisting algorithm (continuous, chattering-free).
    %
    %   u_r = -k1 * |s|^(1/2) * sign(s) + v
    %   vdot = -k2 * sign(s)
    %
    %   Second-order sliding mode reaching law. Produces a CONTINUOUS
    %   control signal (no chattering) while maintaining finite-time
    %   convergence. The gold standard for chattering elimination.
    %
    %   Reference: Levant, A. (1993). "Sliding order and sliding
    %   accuracy in sliding mode control." IJOC, 58(6), 1247-1263.

    properties
        name = 'SuperTwisting'
    end

    methods
        function obj = SuperTwisting(varargin)
            p.k1 = 10;
            p.k2 = 5;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
            obj.state  = struct('v', 0);
        end

        function u_r = compute(obj, s, ~)
            % If state.v not initialized to correct size, fix it
            if numel(obj.state.v) ~= numel(s)
                obj.state.v = zeros(size(s));
            end
            u_r = -obj.params.k1 * abs(s).^0.5 .* sign(s) + obj.state.v;
        end

        function update_state(obj, s, dt)
            %UPDATE_STATE Integrate the super-twisting internal state.
            %   Must be called each time step after compute().
            if numel(obj.state.v) ~= numel(s)
                obj.state.v = zeros(size(s));
            end
            obj.state.v = obj.state.v - obj.params.k2 * sign(s) * dt;
        end

        function reset(obj)
            obj.state = struct('v', 0);
        end
    end
end
