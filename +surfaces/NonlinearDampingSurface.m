classdef NonlinearDampingSurface < core.SlidingSurface
    %NONLINEARDAMPINGSURFACE Nonlinear surface with variable damping ratio.
    %
    %   Implements Bandyopadhyay's variable damping ratio surface that
    %   adapts the effective damping as the output approaches the setpoint:
    %
    %       s = [F - Psi(y)*A12'*P, 1] * [z1; z2]
    %
    %   Simplified continuous-time form (for general 2nd-order systems):
    %       s = edot + (c + Psi(y)) * e
    %
    %   Psi(y) modulates the surface slope from an initial low-damping
    %   configuration (fast response) to a final high-damping configuration
    %   (no overshoot) as the output converges.
    %
    %   Psi function (Eq. 2.7, exponential-quadratic):
    %       Psi(y) = -beta / (1 - e^-1) * (exp(-(1-((y-y0)/(-y0))^2)) - e^-1)
    %
    %   Alternative Psi (Eq. 2.8, Gaussian):
    %       Psi(y) = -beta * exp(-k_tilde * y^2)
    %
    %   Parameters:
    %       c         - (scalar) base surface slope (default 10)
    %       beta      - (scalar) Psi amplitude (default 7.9)
    %       psi_type  - (string) 'exponential'|'gaussian' (default 'exponential')
    %       k_tilde   - (scalar) Gaussian width (for psi_type='gaussian', default 1)
    %       y0        - (scalar) initial output (default 0)
    %       y_ref     - (scalar) reference output (default 1)
    %
    %   Reference: Bandyopadhyay, B. et al. (2009). "Sliding Mode Control
    %   Using Novel Sliding Surfaces", LNCIS 392, Ch 2, Springer.

    properties
        name = 'NonlinearDamping'
    end

    methods
        function obj = NonlinearDampingSurface(varargin)
            p.c        = 10;
            p.beta     = 7.9;
            p.psi_type = 'exponential';
            p.k_tilde  = 1;
            p.y0       = 0;
            p.y_ref    = 1;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;
        end

        function s = compute(obj, e, edot, ~, ~)
            p = obj.params;

            % Use first element for SISO; for MIMO, extend as needed
            y_val = p.y_ref - e(1);  % current output estimate

            % Compute Psi(y)
            psi_val = obj.compute_psi(y_val);

            % Nonlinear sliding surface
            % s = edot + (c + psi) * e
            s = edot + (p.c + psi_val) * e;
        end
    end

    methods (Access = private)
        function psi = compute_psi(obj, y)
            %COMPUTE_PSI Evaluate the nonlinear damping function.
            p = obj.params;
            switch p.psi_type
                case 'exponential'
                    % Eq. 2.7: Psi(y) = -beta/(1-e^-1) * (exp(-(1-((y-y0)/(-y0))^2)) - e^-1)
                    y0 = p.y0;
                    if abs(y0) < 1e-10
                        % Avoid division by zero; use error-based form
                        psi = -p.beta * exp(-p.k_tilde * y^2);
                    else
                        ratio = (y - y0) / (-y0);
                        exponent = -(1 - ratio^2);
                        psi = -p.beta / (1 - exp(-1)) * (exp(exponent) - exp(-1));
                    end
                case 'gaussian'
                    % Eq. 2.8: Psi(y) = -beta * exp(-k_tilde * y^2)
                    psi = -p.beta * exp(-p.k_tilde * y^2);
                otherwise
                    psi = 0;
            end
        end
    end
end
