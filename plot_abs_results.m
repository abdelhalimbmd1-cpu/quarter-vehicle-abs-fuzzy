%% plot_abs_results.m - Plots Quarter-Vehicle ABS Results Matching Fig. 8 (a-g)
% Reference: Geleta, Gopal & Lemi (2024), Fig. 8 (a-g), pp. 505-506
% Reproduces:
%   (a) Slip ratio
%   (b) Wheel velocity (rad/s)
%   (c) Angular vehicle speed [Vehicle Velocity (rad/s)]
%   (d) Vehicle linear velocity [Vehicle speed (m/s)]
%   (e) Stopping distance (m)
%   (f) Brake force (N)
%   (g) Vehicle deceleration (m/s^2)

if ~exist('model_name', 'var') || isempty(model_name)
    model_name = 'abs_fuzzy_model';
end

    % Load parameters
    params_abs;

    % Build model if SLX file does not exist
    slx_file = [model_name '.slx'];
    if ~exist(slx_file, 'file')
        if ~exist('abs_fuzzy.fis', 'file')
            create_abs_fis('abs_fuzzy.fis');
        end
        build_abs_model(model_name);
    end

    % Run simulation
    fprintf('Running simulation of %s...\n', model_name);
    load_system(model_name);
    simOut = sim(model_name);
    fprintf('Simulation completed at t = %.3f s\n', simOut.tout(end));

    % Extract logged signals from logsout
    logs = simOut.logsout;
    t           = simOut.tout;
    slip_data   = logs.get('slip').Values.Data;
    t_slip      = logs.get('slip').Values.Time;

    wspd_data   = logs.get('wheel_speed').Values.Data;
    t_wspd      = logs.get('wheel_speed').Values.Time;

    vel_data    = logs.get('velocity').Values.Data;
    t_vel       = logs.get('velocity').Values.Time;

    dist_data   = logs.get('stopping_distance').Values.Data;
    t_dist      = logs.get('stopping_distance').Values.Time;

    Fx_data     = logs.get('Fx').Values.Data;
    t_Fx        = logs.get('Fx').Values.Time;

    vdot_data   = logs.get('v_dot').Values.Data;
    t_vdot      = logs.get('v_dot').Values.Time;

    % Angular vehicle speed: omega_v = v / R (rad/s)
    w_veh_data  = vel_data / R;

    %% Create Figures Matching Fig. 8 Layout & Styling
    fig = figure('Name', 'Fig. 8 Simulation Results of ABS with FLC at 60 km/h', ...
                 'Color', 'w', 'Position', [100, 50, 1100, 850], 'Visible', 'off');

    % (a) Slip ratio
    subplot(4, 2, 1);
    plot(t_slip, slip_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Slip ratio', 'FontSize', 10, 'Color', 'k');
    title('a) Slip ratio', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 10]); ylim([-0.2 1.1]);
    box on;

    % (b) Wheel velocity (rad/s)
    subplot(4, 2, 2);
    plot(t_wspd, wspd_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Wheel velocity (rad/s)', 'FontSize', 10, 'Color', 'k');
    title('b) Wheel Velocity', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 8]); ylim([0 65]);
    box on;

    % (c) Angular vehicle speed (rad/s)
    subplot(4, 2, 3);
    plot(t_vel, w_veh_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Vehicle Velocity (rad/s)', 'FontSize', 10, 'Color', 'k');
    title('c) Angular vehicle speed', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 8]); ylim([0 65]);
    box on;

    % (d) Vehicle linear velocity (m/s)
    subplot(4, 2, 4);
    plot(t_vel, vel_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Vehicle speed (m/s)', 'FontSize', 10, 'Color', 'k');
    title('d) Vehicle linear velocity', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 10]); ylim([0 18]);
    box on;

    % (e) Stopping distance (m)
    subplot(4, 2, 5);
    plot(t_dist, dist_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Stopping distance (m)', 'FontSize', 10, 'Color', 'k');
    title('e) Stopping distance', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 10]); ylim([0 85]);
    box on;

    % (f) Brake force (N)
    subplot(4, 2, 6);
    plot(t_Fx, Fx_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Brake force (N)', 'FontSize', 10, 'Color', 'k');
    title('f) Brake force', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 10]); ylim([0 950]);
    box on;

    % (g) Vehicle deceleration (m/s^2)
    subplot(4, 2, 7);
    plot(t_vdot, vdot_data, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10, 'Color', 'k');
    ylabel('Vehicle deceleration (m/s2)', 'FontSize', 10, 'Color', 'k');
    title('g) Vehicle deceleration', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.35);
    xlim([0 8]); ylim([-2.7 -0.4]);
    box on;

    % Save plot figure to disk
    this_dir = fileparts(mfilename('fullpath'));
    if isempty(this_dir), this_dir = pwd; end
    plot_file = fullfile(this_dir, 'fig8_results.png');
    saveas(fig, plot_file);
    fprintf('Saved Fig. 8 comparison plots to %s\n', plot_file);

    %% Numerical Verification Summary
    t_stop = t(end);
    final_dist = dist_data(end);
    % Steady state plateau average between t=2.0s and t=6.5s
    idx_ss = (t >= 2.0 & t <= 6.5);
    ss_slip = mean(slip_data(idx_ss));
    ss_Fx   = mean(Fx_data(idx_ss));
    ss_decel= mean(vdot_data(idx_ss));

    fprintf('\n================== NUMERICAL VERIFICATION ==================\n');
    fprintf('Metric                  Target (Fig. 8)       Model Result\n');
    fprintf('------------------------------------------------------------\n');
    fprintf('Stopping Time:          ~ 7.3 s               %.3f s\n', t_stop);
    fprintf('Stopping Distance:      ~ 65 m                %.2f m\n', final_dist);
    fprintf('Settled Slip Ratio:     ~ 0.20                %.3f\n', ss_slip);
    fprintf('Brake Force Plateau:    ~ 830 N               %.1f N\n', ss_Fx);
    fprintf('Deceleration Plateau:   ~ -2.4 m/s^2          %.2f m/s^2\n', ss_decel);
    fprintf('Initial Wheel Velocity: ~ 53 rad/s            %.2f rad/s\n', wspd_data(1));
    fprintf('Initial Vehicle Speed:  ~ 52 rad/s            %.2f rad/s\n', w_veh_data(1));
    fprintf('============================================================\n\n');
