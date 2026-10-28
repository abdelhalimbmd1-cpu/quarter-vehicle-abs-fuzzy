%% simulate_abs_vs_nonabs.m - ABS Active vs. Non-ABS (Wheel Lock) Comparison
% Compares quarter-vehicle emergency braking with Fuzzy ABS vs. Constant Hydraulic Braking (Lockup)
% Outputs high-contrast PNG figure with pure white background and black text.

function simulate_abs_vs_nonabs(model_name)
    if nargin < 1 || isempty(model_name)
        model_name = 'abs_fuzzy_model';
    end

    this_dir = fileparts(mfilename('fullpath'));
    if isempty(this_dir), this_dir = pwd; end
    cd(this_dir);

    % Load parameters
    params_abs;

    %% 1. Run ABS Simulation
    fprintf('Simulating with Fuzzy ABS Controller...\n');
    load_system(model_name);
    simOut_abs = sim(model_name);
    logs_abs = simOut_abs.logsout;
    t_abs = simOut_abs.tout;
    v_abs = logs_abs.get('velocity').Values.Data;
    w_abs = logs_abs.get('wheel_speed').Values.Data;
    s_abs = logs_abs.get('slip').Values.Data;
    Fx_abs = logs_abs.get('Fx').Values.Data;
    dist_abs = logs_abs.get('stopping_distance').Values.Data;
    vdot_abs = logs_abs.get('v_dot').Values.Data;

    %% 2. Run Non-ABS Simulation (Wheel Lockup: Constant Line Pressure P = 2.5 MPa)
    fprintf('Simulating Non-ABS Panic Braking (Lockup)...\n');
    % Numerically solve identical vehicle-wheel differential equations without modulation:
    dt = solver_step;
    t_max = 12.0;
    N = round(t_max / dt);
    
    t_non = zeros(N, 1);
    v_non = zeros(N, 1);
    w_non = zeros(N, 1);
    s_non = zeros(N, 1);
    Fx_non = zeros(N, 1);
    dist_non = zeros(N, 1);
    vdot_non = zeros(N, 1);

    % Initial conditions
    v_curr = v0;
    w_curr = omega0;
    d_curr = 0;
    Tb_non = Kb * 2.5e6 / 7.2 * 1.5; % Locked braking torque

    step = 1;
    t_curr = 0;
    while t_curr < t_max && v_curr > v_stop
        t_non(step) = t_curr;
        v_non(step) = v_curr;
        w_non(step) = w_curr;
        dist_non(step) = d_curr;

        % Slip
        slip_curr = max(0, min(1, (v_curr - w_curr * R) / max(v_curr, 1e-5)));
        s_non(step) = slip_curr;

        % Mu from lookup table
        mu_curr = interp1(slip_table, mu_table, slip_curr, 'linear', 'extrap');
        
        % Deceleration & load transfer
        % Iterative solve for Fl and v_dot
        Fa_curr = 0.5 * rho * Cd * Af * v_curr^2;
        Fr_curr = Fr_val;
        % Fz = Fz0 + Fl where Fl = -m*hcg*v_dot/(2L) and m*v_dot = -(mu*Fz + Fa + Fr)
        % m*v_dot = -(mu*(Fz0 - m*hcg*v_dot/(2L)) + Fa + Fr)
        % v_dot * (m - mu*m*hcg/(2L)) = -(mu*Fz0 + Fa + Fr)
        denom = m - mu_curr * m * hcg / (2 * L);
        v_dot_curr = -(mu_curr * Fz0 + Fa_curr + Fr_curr) / denom;
        Fl_curr = -m * hcg * v_dot_curr / (2 * L);
        Fz_curr = Fz0 + Fl_curr;
        Fx_curr = mu_curr * Fz_curr;

        Fx_non(step) = Fx_curr;
        vdot_non(step) = v_dot_curr;

        % Wheel angular deceleration
        w_dot_curr = (Fx_curr * R - Tb_non) / Iw;

        % Forward Euler / RK update
        v_curr = max(v_stop, v_curr + v_dot_curr * dt);
        w_curr = max(0, w_curr + w_dot_curr * dt);
        d_curr = d_curr + v_curr * dt;
        t_curr = t_curr + dt;
        step = step + 1;
    end

    % Truncate unused array entries
    t_non = t_non(1:step-1);
    v_non = v_non(1:step-1);
    w_non = w_non(1:step-1);
    s_non = s_non(1:step-1);
    Fx_non = Fx_non(1:step-1);
    dist_non = dist_non(1:step-1);
    vdot_non = vdot_non(1:step-1);

    %% Plot ABS vs. Non-ABS with Pure White Background and Sharp Black Text
    fig = figure('Name', 'ABS vs. Non-ABS Emergency Braking Performance', ...
                 'Color', 'w', 'Position', [80, 50, 1150, 850], 'Visible', 'off');

    c_abs = [0.85 0.1 0.1];  % Red
    c_non = [0.15 0.15 0.15];% Black / Dark Gray

    % 1. Speeds
    subplot(2, 2, 1); hold on;
    plot(t_abs, v_abs, 'Color', c_abs, 'LineWidth', 2.0, 'DisplayName', 'ABS Vehicle Speed');
    plot(t_abs, w_abs * R, '--', 'Color', c_abs, 'LineWidth', 1.5, 'DisplayName', 'ABS Wheel Speed (R\omega)');
    plot(t_non, v_non, 'Color', c_non, 'LineWidth', 2.0, 'DisplayName', 'Non-ABS Vehicle Speed');
    plot(t_non, w_non * R, ':', 'Color', [0.3 0.3 0.8], 'LineWidth', 2.0, 'DisplayName', 'Non-ABS Wheel (Locks at t=0.1s)');
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Velocity (m/s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('a) Velocity Comparison: ABS Control vs. Wheel Lockup', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'northeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 9]); ylim([0 18]); box on;

    % 2. Slip Ratio
    subplot(2, 2, 2); hold on;
    yline(0.20, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Optimal Slip (\lambda = 0.20)');
    plot(t_abs, s_abs, 'Color', c_abs, 'LineWidth', 2.0, 'DisplayName', 'ABS Slip (Regulated \approx 0.20)');
    plot(t_non, s_non, 'Color', c_non, 'LineWidth', 2.0, 'DisplayName', 'Non-ABS Slip (100% Locked Skid)');
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Relative Slip Ratio \lambda', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('b) Wheel Slip Ratio: Controlled vs. Skidding', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'east', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 9]); ylim([-0.05 1.1]); box on;

    % 3. Deceleration
    subplot(2, 2, 3); hold on;
    plot(t_abs, vdot_abs, 'Color', c_abs, 'LineWidth', 2.0, 'DisplayName', 'ABS Deceleration');
    plot(t_non, vdot_non, 'Color', c_non, 'LineWidth', 2.0, 'DisplayName', 'Non-ABS Deceleration (Reduced Grip)');
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Vehicle Deceleration (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('c) Deceleration Comparison', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'southeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 9]); box on;

    % 4. Stopping Distance
    subplot(2, 2, 4); hold on;
    plot(t_abs, dist_abs, 'Color', c_abs, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('ABS Stopping Distance: %.1f m', dist_abs(end)));
    plot(t_non, dist_non, 'Color', c_non, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Non-ABS Distance: %.1f m (+%.1f%%)', dist_non(end), ...
        (dist_non(end) - dist_abs(end)) / dist_abs(end) * 100));
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Stopping Distance (m)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title('d) Stopping Distance Comparison', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    grid on; grid minor;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.3 0.5 0.8], 'GridAlpha', 0.4);
    legend('Location', 'northwest', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.7 0.7 0.7]);
    xlim([0 9]); box on;

    % Save High-Resolution Figure
    outfile = fullfile(this_dir, 'fig_option2_abs_vs_nonabs.png');
    exportgraphics(fig, outfile, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved Option 2 ABS vs Non-ABS figure to %s\n', outfile);
end
