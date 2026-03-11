classdef DirectSMC < core.Architecture
    %DIRECTSMC Single-loop architecture for fully actuated systems.
    %
    %   The simplest architecture: one controller directly drives the plant.
    %   Use for fully actuated systems (robot arms, nanopositioners, etc.)

    properties
        name = 'Direct'
    end

    methods
        function obj = DirectSMC(controller)
            obj.controllers = {controller};
            obj.params = struct();
        end

        function [u, info] = compute(obj, t, x, xref, plant)
            [u, info] = obj.controllers{1}.compute(t, x, xref, plant);
        end
    end
end
