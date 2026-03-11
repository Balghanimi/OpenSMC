classdef (Abstract) SlidingSurface < handle
    %SLIDINGSURFACE Abstract base class for all sliding surface designs.
    %
    %   Every SMC variant defines a sliding variable s = f(e, edot, ...).
    %   Subclasses implement the specific surface equation.
    %
    %   The modular design allows swapping surfaces while keeping the same
    %   reaching law, plant, and estimator -- enabling fair benchmarking.
    %
    %   Usage:
    %       surface = surfaces.LinearSurface('c', 10);
    %       s = surface.compute(e, edot, eint, t);

    properties (Abstract)
        name    % (string) Human-readable identifier, e.g. 'Linear', 'Terminal'
    end

    properties
        params  % (struct) Surface-specific parameters
        state   % (struct) Internal state (integrators, memory, etc.)
    end

    methods (Abstract)
        %COMPUTE Evaluate the sliding variable.
        %   s = compute(obj, e, edot, eint, t)
        %
        %   Inputs:
        %       e     - (n x 1) tracking error vector
        %       edot  - (n x 1) error derivative vector
        %       eint  - (n x 1) error integral vector
        %       t     - (scalar) current time
        %
        %   Output:
        %       s     - (n x 1) sliding variable vector
        s = compute(obj, e, edot, eint, t)
    end

    methods
        function reset(obj)
            %RESET Clear internal state for a new simulation run.
            obj.state = struct();
        end

        function info = describe(obj)
            %DESCRIBE Return a struct summarizing the surface for logging.
            info.name   = obj.name;
            info.params = obj.params;
            info.class  = class(obj);
        end
    end
end
