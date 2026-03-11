classdef HierarchicalSurface < core.SlidingSurface
    %HIERARCHICALSURFACE Hierarchical sliding surface for underactuated systems.
    %
    %   Decomposes the system into actuated (s1) and unactuated (s2)
    %   subsystems, then combines them:
    %
    %   s1 = edot_a + c1 * e_a          (actuated subsystem surface)
    %   s2 = edot_u + c2 * e_u          (unactuated subsystem surface)
    %   S  = s1 + lambda * s2           (hierarchical combination)
    %
    %   The parameter lambda controls how strongly the controller
    %   prioritizes the unactuated DOF.
    %
    %   Reference: Unknown (2015). "Hierarchical Sliding Mode Control
    %   for Under-actuated Cranes: Design, Analysis and Simulation."
    %
    %   Setup: Provide indices for actuated and unactuated states.

    properties
        name = 'Hierarchical'
    end

    methods
        function obj = HierarchicalSurface(varargin)
            p.c1     = 10;     % actuated surface slope
            p.c2     = 10;     % unactuated surface slope
            p.lambda = 1;      % coupling weight
            p.idx_a  = [];     % indices of actuated position states in e
            p.idx_u  = [];     % indices of unactuated position states in e
            p.idx_adot = [];   % indices of actuated velocity states in edot
            p.idx_udot = [];   % indices of unactuated velocity states in edot
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            p = obj.params;
            % Actuated subsystem surface
            s1 = edot(p.idx_adot) + p.c1 * e(p.idx_a);
            % Unactuated subsystem surface
            s2 = edot(p.idx_udot) + p.c2 * e(p.idx_u);
            % Hierarchical combination
            s = s1 + p.lambda * s2;
        end
    end
end
