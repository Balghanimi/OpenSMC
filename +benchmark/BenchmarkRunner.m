classdef BenchmarkRunner < handle
    %BENCHMARKRUNNER Run all combinations of architectures x plants.
    %
    %   Usage:
    %       runner = benchmark.BenchmarkRunner();
    %       runner.add_architecture('ClassicalSMC', arch_classical);
    %       runner.add_architecture('NFTSMC', arch_nftsmc);
    %       runner.add_plant('DoubleIntegrator', plant_di, ref_fn, dist_fn);
    %       runner.add_plant('Crane', plant_crane, ref_fn_crane, dist_fn_crane);
    %       results = runner.run_all();
    %       runner.print_table(results);
    %       runner.plot_comparison(results, 'DoubleIntegrator');

    properties
        architectures  % (struct array) .name, .arch
        plants         % (struct array) .name, .plant, .ref_fn, .dist_fn
        simulator      % (benchmark.Simulator)
    end

    methods
        function obj = BenchmarkRunner(varargin)
            obj.architectures = struct('name', {}, 'arch', {});
            obj.plants        = struct('name', {}, 'plant', {}, ...
                                       'ref_fn', {}, 'dist_fn', {});
            obj.simulator     = benchmark.Simulator(varargin{:});
        end

        function add_architecture(obj, name, arch)
            entry.name = name;
            entry.arch = arch;
            obj.architectures(end+1) = entry;
        end

        function add_plant(obj, name, plant, ref_fn, dist_fn)
            entry.name    = name;
            entry.plant   = plant;
            entry.ref_fn  = ref_fn;
            entry.dist_fn = dist_fn;
            obj.plants(end+1) = entry;
        end

        function results = run_all(obj)
            %RUN_ALL Execute every architecture on every plant.
            %   Returns a struct array: results(i,j) for arch i, plant j.
            na = numel(obj.architectures);
            np = numel(obj.plants);

            results = struct('arch_name', {}, 'plant_name', {}, ...
                             'result', {}, 'metrics', {});
            results(na, np).arch_name = '';

            for i = 1:na
                for j = 1:np
                    fprintf('Running %s on %s...\n', ...
                        obj.architectures(i).name, obj.plants(j).name);

                    res = obj.simulator.run( ...
                        obj.architectures(i).arch, ...
                        obj.plants(j).plant, ...
                        obj.plants(j).ref_fn, ...
                        obj.plants(j).dist_fn);

                    met = benchmark.Metrics.compute_all(res);

                    results(i,j).arch_name  = obj.architectures(i).name;
                    results(i,j).plant_name = obj.plants(j).name;
                    results(i,j).result     = res;
                    results(i,j).metrics    = met;
                end
            end
        end

        function print_table(~, results)
            %PRINT_TABLE Print comparison table to console.
            [na, np] = size(results);

            for j = 1:np
                fprintf('\n=== Plant: %s ===\n', results(1,j).plant_name);
                fprintf('%-20s %10s %10s %10s %10s %12s %12s\n', ...
                    'Controller', 'RMSE', 'MAE', 'Ts (s)', ...
                    'OS (%)', 'Effort', 'Chatter');
                fprintf('%s\n', repmat('-', 1, 86));

                for i = 1:na
                    m = results(i,j).metrics;
                    fprintf('%-20s %10.4f %10.4f %10.4f %10.2f %12.2f %12.2f\n', ...
                        results(i,j).arch_name, ...
                        m.rmse, m.mae, m.settling_time, ...
                        m.overshoot, m.control_effort, m.chattering_idx);
                end
            end
        end

        function plot_comparison(~, results, plant_name)
            %PLOT_COMPARISON Plot tracking and sliding variables for one plant.
            [na, np] = size(results);

            % Find plant column
            j = find(arrayfun(@(r) strcmp(r.plant_name, plant_name), ...
                results(1,:)), 1);
            if isempty(j)
                error('Plant "%s" not found.', plant_name);
            end

            figure('Name', ['Comparison: ' plant_name], ...
                   'Position', [100 100 1200 800]);

            % Tracking
            subplot(3,1,1); hold on;
            plot(results(1,j).result.t, results(1,j).result.xref(1,:), ...
                'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
            for i = 1:na
                plot(results(i,j).result.t, results(i,j).result.x(1,:), ...
                    'LineWidth', 1.5, 'DisplayName', results(i,j).arch_name);
            end
            ylabel('Output'); title(['Tracking: ' plant_name]);
            legend('Location', 'best'); grid on;

            % Control signal
            subplot(3,1,2); hold on;
            for i = 1:na
                plot(results(i,j).result.t, results(i,j).result.u(1,:), ...
                    'LineWidth', 1, 'DisplayName', results(i,j).arch_name);
            end
            ylabel('Control u'); title('Control Signal');
            legend('Location', 'best'); grid on;

            % Sliding variable
            subplot(3,1,3); hold on;
            for i = 1:na
                if ~isempty(results(i,j).result.s)
                    plot(results(i,j).result.t, results(i,j).result.s(1,:), ...
                        'LineWidth', 1, 'DisplayName', results(i,j).arch_name);
                end
            end
            xlabel('Time (s)'); ylabel('s(t)');
            title('Sliding Variable'); legend('Location', 'best'); grid on;
        end
    end
end
