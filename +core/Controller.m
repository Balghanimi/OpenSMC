classdef (Abstract) Controller < handle
    %CONTROLLER Abstract base class for SMC controllers.
    %
    %   A Controller combines a SlidingSurface, ReachingLaw, and optionally
    %   an Estimator into a complete control law. It also computes the
    %   equivalent control (model-based feedforward) when applicable.
    %
    %   For direct SMC (fully actuated systems), one Controller suffices.
    %   For cascaded or hierarchical architectures, multiple Controllers
    %   are composed by an Architecture object.

    properties (Abstract)
        name    % (string) e.g. 'ClassicalSMC', 'NFTSMC', 'ITSMC'
    end

    properties
        surface     % (core.SlidingSurface) sliding surface object
        reaching    % (core.ReachingLaw) reaching law object
        estimator   % (core.Estimator) disturbance estimator (or [])
        params      % (struct) controller-level parameters
        state       % (struct) internal state (integrators, etc.)
    end

    methods (Abstract)
        %COMPUTE Full control law.
        %   [u, info] = compute(obj, t, x, xref, plant)
        %
        %   Inputs:
        %       t     - (scalar) time
        %       x     - (n x 1) current state
        %       xref  - (n x 1) reference state
        %       plant - (core.Plant) plant object (for equivalent control)
        %
        %   Outputs:
        %       u    - (m x 1) control input
        %       info - (struct) diagnostics: .s, .ueq, .ur, .dhat, etc.
        [u, info] = compute(obj, t, x, xref, plant)
    end

    methods
        function reset(obj)
            obj.state = struct();
            if ~isempty(obj.surface),   obj.surface.reset();   end
            if ~isempty(obj.reaching),  obj.reaching.reset();  end
            if ~isempty(obj.estimator), obj.estimator.reset(); end
        end

        function info = describe(obj)
            info.name      = obj.name;
            info.params    = obj.params;
            info.class     = class(obj);
            info.surface   = obj.surface.describe();
            info.reaching  = obj.reaching.describe();
            if ~isempty(obj.estimator)
                info.estimator = obj.estimator.describe();
            else
                info.estimator = 'None';
            end
        end
    end
end
