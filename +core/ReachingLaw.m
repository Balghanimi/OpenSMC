classdef (Abstract) ReachingLaw < handle
    %REACHINGLAW Abstract base class for reaching law designs.
    %
    %   The reaching law determines how the system state is driven toward
    %   the sliding surface. Common choices: constant-rate, exponential,
    %   power-rate, super-twisting, saturation (boundary layer).
    %
    %   Separated from the surface so the same reaching law can be paired
    %   with different surfaces and vice versa.

    properties (Abstract)
        name    % (string) Identifier, e.g. 'ConstantRate', 'SuperTwisting'
    end

    properties
        params  % (struct) Reaching law parameters
        state   % (struct) Internal state (e.g. super-twisting integrator)
    end

    methods (Abstract)
        %COMPUTE Evaluate the reaching/discontinuous control component.
        %   u_r = compute(obj, s, t)
        %
        %   Inputs:
        %       s   - (n x 1) sliding variable
        %       t   - (scalar) current time
        %
        %   Output:
        %       u_r - (n x 1) reaching control term
        u_r = compute(obj, s, t)
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
