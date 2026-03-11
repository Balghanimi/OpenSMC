classdef disturbances
    %DISTURBANCES Standard disturbance profiles for benchmarking.
    %
    %   Usage:
    %       dist_fn = utils.disturbances.step(0.5, 2.0);  % step of 0.5 at t=2
    %       dist_fn = utils.disturbances.sinusoidal(1.0, 5.0);
    %       dist_fn = utils.disturbances.composite({dist1, dist2});

    methods (Static)

        function fn = none(n)
            %NONE Zero disturbance.
            if nargin < 1, n = 2; end
            fn = @(t) zeros(n, 1);
        end

        function fn = step(amplitude, onset_time, n)
            %STEP Step disturbance at specified time.
            if nargin < 3, n = 2; end
            fn = @(t) (t >= onset_time) * amplitude * ones(n, 1);
        end

        function fn = sinusoidal(amplitude, frequency, n)
            %SINUSOIDAL Persistent sinusoidal disturbance.
            if nargin < 3, n = 2; end
            fn = @(t) amplitude * sin(2*pi*frequency*t) * ones(n, 1);
        end

        function fn = random_band(amplitude, seed, n)
            %RANDOM_BAND Band-limited random disturbance (reproducible).
            if nargin < 3, n = 2; end
            rng(seed);
            % Pre-generate random sequence
            N = 100000;
            noise = amplitude * randn(n, N);
            fn = @(t) noise(:, max(1, min(N, round(t*1000)+1)));
        end

        function fn = composite(dist_fns)
            %COMPOSITE Sum of multiple disturbances.
            fn = @(t) composite_eval(t, dist_fns);
        end
    end
end

function d = composite_eval(t, fns)
    d = fns{1}(t);
    for i = 2:numel(fns)
        d = d + fns{i}(t);
    end
end
