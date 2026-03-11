classdef RBF_ELM < core.Estimator
    %RBF_ELM RBF-ELM hybrid neural network disturbance estimator.
    %
    %   Gaussian hidden layer (fixed centers/widths) + ELM output weights.
    %   Online adaptation via OS-ELM (Online Sequential ELM) using
    %   Woodbury matrix identity for efficient rank-1 covariance updates.
    %
    %   Architecture:
    %       Input x -> [h1(x), h2(x), ..., hL(x)] -> W_out * h(x) = d_hat
    %
    %   Key properties:
    %       - Centers and widths are FIXED (no gradient tuning)
    %       - Output weights updated online via Lyapunov-based OS-ELM
    %       - Deterministic initialization (no iterative training)
    %       - Suitable for real-time disturbance estimation in SMC
    %
    %   Parameters:
    %       n_hidden  - (int) number of RBF neurons (default 25)
    %       x_min     - (1 x n_input) lower bounds of input space (default [-3 -5])
    %       x_max     - (1 x n_input) upper bounds of input space (default [3 5])
    %       lambda    - (scalar) regularization parameter (default 1e-4)
    %
    %   Reference: Huang, G.-B. et al. (2006). "Extreme Learning Machine:
    %   Theory and Applications." Neurocomputing, 70(1-3), 489-501.
    %   Also: Liang, N.-Y. et al. (2006). "A Fast and Accurate Online
    %   Sequential Learning Algorithm for Feedforward Networks."

    properties
        name = 'RBF_ELM'
    end

    methods
        function obj = RBF_ELM(varargin)
            p.n_hidden = 25;
            p.x_min    = [-3, -5];
            p.x_max    = [3, 5];
            p.lambda   = 1e-4;
            for i = 1:2:numel(varargin)
                p.(varargin{i}) = varargin{i+1};
            end
            obj.params = p;

            n_input  = numel(p.x_min);
            n_hidden = p.n_hidden;

            % Place centers uniformly (Latin Hypercube Sampling)
            centers = zeros(n_hidden, n_input);
            for d = 1:n_input
                centers(:,d) = p.x_min(d) + (p.x_max(d) - p.x_min(d)) * ...
                    ((randperm(n_hidden)' - 0.5) / n_hidden);
            end

            % Nearest-neighbor width heuristic
            widths = zeros(n_hidden, 1);
            for j = 1:n_hidden
                dists = sqrt(sum((centers - centers(j,:)).^2, 2));
                dists(j) = inf;
                k = min(3, n_hidden-1);
                sorted_d = sort(dists);
                widths(j) = mean(sorted_d(1:k));
            end
            widths = max(widths, 0.01);

            % Initialize state
            obj.state = struct( ...
                'centers', centers, ...
                'widths', widths, ...
                'W_out', zeros(n_hidden, 1), ...
                'P', eye(n_hidden) / p.lambda, ...
                'initialized', false);
        end

        function dhat = estimate(obj, ~, x, ~, ~)
            % Compute disturbance estimate from state vector.
            % Uses first two state elements as RBF input [position, velocity].
            n = numel(x);
            n_input = numel(obj.params.x_min);

            % Extract input features (clamp to operating range)
            x_in = zeros(1, n_input);
            for d = 1:min(n, n_input)
                x_in(d) = max(obj.params.x_min(d), ...
                          min(obj.params.x_max(d), x(d)));
            end

            % Forward pass: compute hidden layer
            h = obj.compute_hidden(x_in);  % [1 x n_hidden]

            % Output: d_hat = h * W_out
            d_hat_scalar = h * obj.state.W_out;

            % Return as full state-sized vector
            dhat = zeros(n, 1);
            dhat(1) = d_hat_scalar;
        end

        function update(obj, ~, x, ~, ~, ~)
            % Online OS-ELM weight update (called externally with target).
            % For ITSMC integration, the controller calls online_update directly.
            % This method exists for interface compliance.
        end

        function online_update(obj, x_in, target)
            %ONLINE_UPDATE OS-ELM sequential weight update.
            %   x_in:   (1 x n_input) input features
            %   target: (scalar or 1 x n_output) learning target

            h = obj.compute_hidden(x_in)';  % [n_hidden x 1]

            % Woodbury identity update
            Ph = obj.state.P * h;
            denom = 1 + h' * Ph;
            obj.state.P = obj.state.P - (Ph * Ph') / denom;

            % Weight update
            error = target(:) - obj.state.W_out' * h;
            obj.state.W_out = obj.state.W_out + obj.state.P * h * error';

            obj.state.initialized = true;
        end

        function y_hat = predict(obj, x_in)
            %PREDICT Forward pass for direct use.
            %   x_in: (1 x n_input) input features
            %   y_hat: (n_output x 1) prediction
            h = obj.compute_hidden(x_in);
            y_hat = h * obj.state.W_out;
        end

        function reset(obj)
            n_hidden = obj.params.n_hidden;
            obj.state.W_out = zeros(n_hidden, 1);
            obj.state.P = eye(n_hidden) / obj.params.lambda;
            obj.state.initialized = false;
        end
    end

    methods (Access = private)
        function h = compute_hidden(obj, x)
            %COMPUTE_HIDDEN Evaluate Gaussian basis functions.
            %   x: (1 x n_input)
            %   h: (1 x n_hidden)
            n_hidden = size(obj.state.centers, 1);
            h = zeros(1, n_hidden);
            for j = 1:n_hidden
                diff = x - obj.state.centers(j,:);
                h(j) = exp(-sum(diff.^2) / (2 * obj.state.widths(j)^2));
            end
        end
    end
end
