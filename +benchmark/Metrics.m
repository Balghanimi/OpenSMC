classdef Metrics
    %METRICS Standardized performance metrics for SMC benchmarking.
    %
    %   All methods are static -- call as benchmark.Metrics.rmse(result).
    %   Every benchmark experiment uses the SAME metrics, ensuring fair
    %   comparison across SMC variants.

    methods (Static)

        function m = compute_all(result, settling_threshold)
            %COMPUTE_ALL Compute all standard metrics from a simulation result.
            %
            %   m = Metrics.compute_all(result)
            %   m = Metrics.compute_all(result, settling_threshold)
            %
            %   settling_threshold defaults to 0.02 (2%).

            if nargin < 2, settling_threshold = 0.02; end

            m.rmse            = benchmark.Metrics.rmse(result);
            m.mae             = benchmark.Metrics.mae(result);
            m.ise             = benchmark.Metrics.ise(result);
            m.iae             = benchmark.Metrics.iae(result);
            m.settling_time   = benchmark.Metrics.settling_time(result, settling_threshold);
            m.overshoot       = benchmark.Metrics.overshoot(result);
            m.control_effort  = benchmark.Metrics.control_effort(result);
            m.chattering_idx  = benchmark.Metrics.chattering_index(result);
            m.reaching_time   = benchmark.Metrics.reaching_time(result);
            m.steady_state_error = benchmark.Metrics.steady_state_error(result);
        end

        function v = rmse(result)
            %RMSE Root Mean Square Error of tracking.
            e = result.e;
            % Use position states only (first half for second-order systems)
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            v = sqrt(mean(e(:).^2));
        end

        function v = mae(result)
            %MAE Mean Absolute Error.
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            v = mean(abs(e(:)));
        end

        function v = ise(result)
            %ISE Integral of Squared Error.
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            v = trapz(result.t, sum(e.^2, 1));
        end

        function v = iae(result)
            %IAE Integral of Absolute Error.
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            v = trapz(result.t, sum(abs(e), 1));
        end

        function v = settling_time(result, threshold)
            %SETTLING_TIME Time to enter and stay within threshold of reference.
            if nargin < 2, threshold = 0.02; end
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            err_norm = vecnorm(e, 2, 1);
            ref_norm = max(vecnorm(result.xref(1:size(e,1), :), 2, 1));
            if ref_norm == 0, ref_norm = 1; end

            settled = err_norm / ref_norm < threshold;
            % Find last time it was NOT settled
            idx = find(~settled, 1, 'last');
            if isempty(idx)
                v = 0;
            elseif idx >= numel(result.t)
                v = Inf;
            else
                v = result.t(idx + 1);
            end
        end

        function v = overshoot(result)
            %OVERSHOOT Maximum percentage overshoot.
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            % Normalize by max absolute reference (robust to sinusoidal/zero-crossing refs)
            ref_pos = result.xref(1:size(e,1), :);
            ref_scale = max(abs(ref_pos(:)));
            if ref_scale < 1e-10
                v = 0;
                return;
            end
            % Overshoot = max deviation beyond reference, per time step
            x_pos = result.x(1:size(e,1), :);
            overshoot_vals = (x_pos - ref_pos) ./ ref_scale;
            v = max(overshoot_vals(:)) * 100;  % percentage
            v = max(v, 0);  % only positive overshoot
        end

        function v = control_effort(result)
            %CONTROL_EFFORT Integral of squared control input (energy proxy).
            v = trapz(result.t, sum(result.u.^2, 1));
        end

        function v = chattering_index(result)
            %CHATTERING_INDEX Measure of control signal high-frequency content.
            %   Higher = more chattering. Computed as total variation of u
            %   normalized by simulation time.
            du = diff(result.u, 1, 2);
            total_variation = sum(abs(du(:)));
            v = total_variation / (result.t(end) - result.t(1));
        end

        function v = reaching_time(result)
            %REACHING_TIME Time for sliding variable to first reach zero.
            if isempty(result.s)
                v = NaN;
                return;
            end
            s_norm = vecnorm(result.s, 2, 1);
            threshold = 0.01 * max(s_norm);
            idx = find(s_norm < threshold, 1, 'first');
            if isempty(idx)
                v = Inf;
            else
                v = result.t(idx);
            end
        end

        function v = steady_state_error(result)
            %STEADY_STATE_ERROR Average error over last 10% of simulation.
            e = result.e;
            n_pos = size(e, 1) / 2;
            if n_pos == floor(n_pos)
                e = e(1:n_pos, :);
            end
            N = size(e, 2);
            tail = e(:, round(0.9*N):N);
            v = mean(vecnorm(tail, 2, 1));
        end

        function v = max_swing(result, angle_idx)
            %MAX_SWING Maximum swing angle [rad] for underactuated systems.
            %   angle_idx: row index of the swing angle in result.x (default 3).
            if nargin < 2, angle_idx = 3; end
            v = max(abs(result.x(angle_idx, :)));
        end

        function v = residual_swing(result, angle_idx)
            %RESIDUAL_SWING RMS swing angle over last 20% of simulation [rad].
            %   angle_idx: row index of the swing angle in result.x (default 3).
            if nargin < 2, angle_idx = 3; end
            N = size(result.x, 2);
            tail = result.x(angle_idx, round(0.8*N):N);
            v = sqrt(mean(tail.^2));
        end
    end
end
