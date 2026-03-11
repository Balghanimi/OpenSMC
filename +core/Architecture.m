classdef (Abstract) Architecture < handle
    %ARCHITECTURE Abstract base class for control architectures.
    %
    %   Architectures define HOW controllers are composed for a given plant.
    %   This is the higher-level structural layer above individual controllers.
    %
    %   Types:
    %     DirectSMC       - Single controller, fully actuated systems
    %     CascadedSMC     - Outer loop (position) + Inner loop (attitude)
    %     HierarchicalSMC - Subsystem decomposition for underactuated systems
    %     ObserverBasedSMC- SMC observer feeds SMC controller (output feedback)
    %
    %   The Architecture owns the controllers and routes signals between them.

    properties (Abstract)
        name    % (string) e.g. 'Direct', 'Cascaded', 'Hierarchical'
    end

    properties
        controllers  % (cell array of core.Controller) sub-controllers
        params       % (struct) architecture-level parameters
    end

    methods (Abstract)
        %COMPUTE Compute the final control input using the architecture.
        %   [u, info] = compute(obj, t, x, xref, plant)
        %
        %   This method orchestrates multiple controllers if needed.
        %   For DirectSMC, it simply delegates to the single controller.
        %   For CascadedSMC, outer loop produces inner reference.
        %   For HierarchicalSMC, surfaces are composed hierarchically.
        [u, info] = compute(obj, t, x, xref, plant)
    end

    methods
        function reset(obj)
            for i = 1:numel(obj.controllers)
                obj.controllers{i}.reset();
            end
        end

        function info = describe(obj)
            info.name        = obj.name;
            info.params      = obj.params;
            info.class       = class(obj);
            info.controllers = cellfun(@(c) c.describe(), ...
                obj.controllers, 'UniformOutput', false);
        end
    end
end
