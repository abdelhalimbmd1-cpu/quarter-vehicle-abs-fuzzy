%% simulate_road_comparison.m - Multi-Road Surface ABS Performance Comparison
% Simulates Quarter-Vehicle ABS on Dry Asphalt, Wet Asphalt, and Icy Road
% Outputs high-contrast PNG figure with pure white background and black text.

function simulate_road_comparison(model_name)
    if nargin < 1 || isempty(model_name)
        model_name = 'abs_fuzzy_model';
    end

    this_dir = fileparts(mfilename('fullpath'));
    if isempty(this_dir), this_dir = pwd; end
    cd(this_dir);

    % Ensure model exists
    if ~exist([model_name '.slx'], 'file')
        build_abs_model(model_name);
    end

    roads = {'dry', 'wet', 'ice'};
    road_labels = {'Dry Asphalt (\mu_{peak} \approx 0.90)', ...
                   'Wet Asphalt (\mu_{peak} \approx 0.45)', ...
                   'Icy Road (\mu_{peak} \approx 0.20)'};
    colors = {[0.85 0.1 0.1], [0.1 0.5 0.8], [0.2 0.7 0.3]}; % Red, Blue, Green

    results = struct();

    for i = 1:length(roads)
        road_condition = roads{i};
        fprintf('Simulating on %s...\n', road_labels{i});
        params_abs;
        sim_time = 15.0; % allow longer time for wet and ice
        
        load_system(model_name);
        simOut = sim(model_name, 'StopTime', num2str(sim_time));
        
        logs = simOut.logsout;
        results(i).name = roads{i};
        results(i).label = road_labels{i};
        results(i).t = simOut.tout;
        results(i).v = logs.get('velocity').Values.Data;
        results(i).w = logs.get('wheel_speed').Values.Data;
        results(i).s = logs.get('slip').Values.Data;
        results(i).Fx = logs.get('Fx').Values.Data;
        results(i).dist = logs.get('stopping_distance').Values.Data;
        results(i).vdot = logs.get('v_dot').Values.Data;
        results(i).t_stop = results(i).t(end);
        results(i).d_stop = results(i).dist(end);
    end

    %% Plot Results with Pure White Background and Sharp Black Text
    fig = figure('Name', 'Quarter-Vehicle ABS Road Surface Comparison', ...
                 'Color', 'w', 'Position', [80, 50, 1150, 850], 'Visible', 'off');

    % 1. Vehicle and Wheel Speeds
    subplot(2, 2, 1); hold on;
    for i = 1:3
        plot(results(i).t, results(i).v, 'Color', colors{i}, 'LineWidth', 2.0, ...
            'DisplayName', [road_labels{i} ' - Veh']);
        plot(results(i).t, results(i).w * R, '--', 'Color', colors{i}, 'LineWidth', 1.2, ...
            'DisplayName', [road_labels{i} ' - Wheel (R\omega)']);
    end
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Velocity (m/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('a) Vehicle Speed (Solid) & Linear Wheel Speed (Dashed)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'northeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 14]); ylim([0 18]); box on;

    % 2. Stopping Distance
    subplot(2, 2, 2); hold on;
    for i = 1:3
        plot(results(i).t, results(i).dist, 'Color', colors{i}, 'LineWidth', 2.0, ...
            'DisplayName', sprintf('%s: %.1f m (%.2f s)', road_labels{i}, results(i).d_stop, results(i).t_stop));
    end
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Stopping Distance (m)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('b) Stopping Distance vs. Time', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'northwest', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 14]); box on;

    % 3. Braking Force Fx
    subplot(2, 2, 3); hold on;
    for i = 1:3
        plot(results(i).t, results(i).Fx, 'Color', colors{i}, 'LineWidth', 2.0, ...
            'DisplayName', road_labels{i});
    end
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Braking Traction Force Fx (N)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('c) Longitudinal Tire Braking Force Fx', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'northeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 14]); ylim([0 1000]); box on;

    % 4. Slip Ratio Regulation
    subplot(2, 2, 4); hold on;
    yline(0.20, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target Slip \lambda_0 = 0.20');
    for i = 1:3
        plot(results(i).t, results(i).s, 'Color', colors{i}, 'LineWidth', 1.8, ...
            'DisplayName', road_labels{i});
    end
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Relative Slip Ratio \lambda', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('d) Longitudinal Slip Regulation', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'southeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 14]); ylim([-0.05 1.05]); box on;

    % Save High-Resolution Figure
    outfile = fullfile(this_dir, 'fig_option1_road_comparison.png');
    exportgraphics(fig, outfile, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved Option 1 road comparison figure to %s\n', outfile);
end
