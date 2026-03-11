classdef DoubleIntegrator < core.Plant
    %DOUBLEINTEGRATOR Simplest benchmark plant for SMC testing.
    %
    %   x1dot = x2
    %   x2dot = u + d
    %
    %   State: x = [position; velocity]
    %   Input: u = force
    %
    %   Every SMC paper should first demonstrate on this system.
    %   If your controller doesn't work here, it won't work anywhere.

    properties
        name             = 'DoubleIntegrator'
        n_states         = 2
        n_inputs         = 1
        n_outputs        = 1
        n_dof            = 1
        is_underactuated = false
    end

    methods
        function obj = DoubleIntegrator(varargin)
            obj.params = struct();  % no physical parameters
            obj.x0     = [0; 0];
            for i = 1:2:numel(varargin)
                if strcmp(varargin{i}, 'x0')
                    obj.x0 = varargin{i+1};
                end
            end
        end

        function xdot = dynamics(~, ~, x, u, d)
            xdot = [x(2); u + d(1)];
        end

        function y = output(~, x)
            y = x(1);
        end

        function [e, edot] = get_error(~, x, xref)
            e    = xref(1) - x(1);
            edot = xref(2) - x(2);
        end
    end
end
