%% build_abs_model.m - Programmatically builds abs_fuzzy_model.slx
% Reference: Geleta, Gopal & Lemi (2024), Fig. 7 Quarter-vehicle model
% Reproduces Fig. 7 layout, block types, names, and signal routing with exact fidelity.

function build_abs_model(model_name)
    if nargin < 1 || isempty(model_name)
        model_name = 'abs_fuzzy_model';
    end

    % Ensure userpath and current working directory are set to this directory
    model_dir = fileparts(mfilename('fullpath'));
    if isempty(model_dir)
        model_dir = pwd;
    end
    userpath(model_dir);
    cd(model_dir);

    % Ensure parameters and FIS are available in workspace and on disk
    params_abs;
    if ~exist('abs_fuzzy.fis', 'file')
        create_abs_fis('abs_fuzzy.fis');
    end

    % Close model if already loaded/open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Create new system
    new_system(model_name);
    open_system(model_name);

    % Configure Model Solver and Logging Parameters
    set_param(model_name, ...
        'SolverType', 'Fixed-step', ...
        'Solver', solver_type, ...
        'FixedStep', num2str(solver_step), ...
        'StartTime', '0.0', ...
        'StopTime', num2str(sim_time), ...
        'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout', ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'SaveTime', 'on', ...
        'TimeSaveName', 'tout');

    %% 1. Desired Slip Constant Block
    b_des = add_block('simulink/Sources/Constant', [model_name '/Desired slip'], ...
        'Value', 'desired_slip', ...
        'Position', [52, 140, 95, 180]);

    %% 2. Sum Block for Slip Error (Round)
    % Error = Actural slip - Desired slip = lambda - 0.2 (ListOfSigns '|-+|')
    b_sum_err = add_block('simulink/Math Operations/Sum', [model_name '/Sum_error'], ...
        'IconShape', 'round', ...
        'ListOfSigns', '|-+|', ...
        'ShowName', 'off', ...
        'Position', [112, 143, 142, 178]);

    %% 3. Error Rate Derivative Block
    b_err_rate = add_block('simulink/Continuous/Derivative', [model_name '/Error rate'], ...
        'Position', [172, 169, 212, 209]);

    %% 4. Mux Block 1 (Error and Error Rate)
    b_mux1 = add_block('simulink/Signal Routing/Mux', [model_name '/Mux1'], ...
        'Inputs', '2', ...
        'ShowName', 'off', ...
        'Position', [239, 137, 245, 219]);

    %% 5. Fuzzy Logic Controller Block
    b_flc = add_block('fuzblock/Fuzzy Logic  Controller', [model_name '/Fuzzy Logic Controller'], ...
        'FIS', '''abs_fuzzy.fis''', ...
        'OutOfRangeInputValueMessage', 'none', ...
        'NoRuleFiredMessage', 'none', ...
        'EmptyOutputFuzzySetMessage', 'none', ...
        'Position', [281, 155, 335, 198]);

    %% 6. Subsystem: Calculating wheel speed
    b_calc_omega = add_block('simulink/Ports & Subsystems/Subsystem', [model_name '/Calculating wheel speed'], ...
        'Position', [451, 158, 574, 230]);
    build_subsystem_wheel_speed([model_name '/Calculating wheel speed']);

    %% 7. Wheel Radius Gain Block (Torque: Fx * R)
    b_wr = add_block('simulink/Math Operations/Gain', [model_name '/Wheel radius'], ...
        'Gain', 'R', ...
        'Position', [388, 172, 434, 206]);

    %% 8. Slip Scope
    b_sc_slip = add_block('simulink/Sinks/Scope', [model_name '/Slip'], ...
        'Position', [153, 246, 186, 276]);

    %% 9. F-load transfer Gain Block
    b_fl = add_block('simulink/Math Operations/Gain', [model_name '/F-load transfer'], ...
        'Gain', 'K_Fl', ...
        'Orientation', 'left', ...
        'Position', [268, 246, 314, 280]);

    %% 10. Speed Derivative Block
    b_sp_der = add_block('simulink/Continuous/Derivative', [model_name '/Speed derivative'], ...
        'Orientation', 'left', ...
        'Position', [418, 246, 458, 286]);

    %% 11. Fr Constant Block (Rolling Resistance)
    b_fr = add_block('simulink/Sources/Constant', [model_name '/Fr'], ...
        'Value', 'Fr_val', ...
        'Position', [254, 301, 288, 335]);

    %% 12. Rectangular Sum Block (Force Summation: -(Fr + Fx + Fa))
    b_sum_force = add_block('simulink/Math Operations/Sum', [model_name '/Sum_force'], ...
        'IconShape', 'rectangular', ...
        'ListOfSigns', '---', ...
        'ShowName', 'off', ...
        'Position', [344, 275, 359, 393]);

    %% 13. (1/m) Vehicle Mass Gain Block
    b_inv_m = add_block('simulink/Math Operations/Gain', [model_name '/(1//m)'], ...
        'Gain', '1/m', ...
        'ShowName', 'off', ...
        'Position', [408, 321, 453, 355]);

    %% 14. Integrator for Vehicle Velocity (with Saturation and Port)
    b_int_v = add_block('simulink/Continuous/Integrator', [model_name '/Integrator_v'], ...
        'InitialCondition', 'v0', ...
        'LimitOutput', 'on', ...
        'LowerSaturationLimit', 'v_stop', ...
        'UpperSaturationLimit', 'inf', ...
        'ShowSaturationPort', 'on', ...
        'ShowName', 'off', ...
        'Position', [489, 319, 526, 355]);

    %% 15. STOP Block (Stop Simulation)
    b_stop = add_block('simulink/Sinks/Stop Simulation', [model_name '/STOP'], ...
        'ShowName', 'off', ...
        'Position', [539, 339, 571, 371]);

    %% 16. Speed Square Math Function Block
    b_sp_sq = add_block('simulink/Math Operations/Math Function', [model_name '/Speed Square'], ...
        'Operator', 'square', ...
        'Orientation', 'left', ...
        'Position', [479, 394, 514, 431]);

    %% 17. Fa Gain Block (Aerodynamic Drag)
    b_fa = add_block('simulink/Math Operations/Gain', [model_name '/Fa'], ...
        'Gain', 'K_Fa', ...
        'Orientation', 'left', ...
        'Position', [394, 397, 439, 431]);

    %% 18. mu vs slip lookup table (1-D Lookup Table)
    b_lut = add_block('simulink/Lookup Tables/1-D Lookup Table', [model_name '/mu vs slip lookup table'], ...
        'Table', 'mu_table', ...
        'BreakpointsForDimension1', 'slip_table', ...
        'Position', [147, 367, 197, 413]);

    %% 19. Product Block (mu * Fl)
    b_prod = add_block('simulink/Math Operations/Product', [model_name '/Product'], ...
        'ShowName', 'off', ...
        'Position', [232, 368, 264, 398]);

    %% 20. Fz Gain Block (Static Normal Force)
    b_fz = add_block('simulink/Math Operations/Gain', [model_name '/Fz'], ...
        'Gain', 'Fz0', ...
        'Position', [228, 419, 274, 453]);

    %% 21. Circle Sum Block for Fx: mu*Fl + mu*Fz0
    b_sum_fx = add_block('simulink/Math Operations/Sum', [model_name '/Sum_Fx'], ...
        'IconShape', 'round', ...
        'ListOfSigns', '|++|', ...
        'ShowName', 'off', ...
        'Position', [284, 372, 314, 402]);

    %% 22. Wheel Radius1 Gain Block (Angular to Linear Wheel Speed)
    b_wr1 = add_block('simulink/Math Operations/Gain', [model_name '/Wheel radius1'], ...
        'Gain', 'R', ...
        'Orientation', 'left', ...
        'Position', [485, 442, 530, 476]);

    %% 23. Subsystem: Actual relative slip (spelled 'Actural slip' per Fig. 7)
    b_calc_slip = add_block('simulink/Ports & Subsystems/Subsystem', [model_name '/Actural slip'], ...
        'Orientation', 'left', ...
        'Position', [282, 462, 462, 525]);
    build_subsystem_slip([model_name '/Actural slip']);

    %% 24. Integrator for Stopping Distance
    b_int_dist = add_block('simulink/Continuous/Integrator', [model_name '/Integrator_dist'], ...
        'InitialCondition', '0', ...
        'ShowName', 'off', ...
        'Position', [586, 396, 617, 432]);

    %% 25. Mux Block 2 (Velocity Scope: Wheel & Vehicle Speeds)
    b_mux2 = add_block('simulink/Signal Routing/Mux', [model_name '/Mux2'], ...
        'Inputs', '2', ...
        'ShowName', 'off', ...
        'Position', [639, 333, 645, 385]);

    %% 26. Velocity Scope
    b_sc_vel = add_block('simulink/Sinks/Scope', [model_name '/Velocity'], ...
        'Position', [665, 345, 696, 374]);

    %% 27. Stopping Distance Scope
    b_sc_dist = add_block('simulink/Sinks/Scope', [model_name '/Stopping distance'], ...
        'Position', [647, 398, 679, 428]);

    %% Get Port Handles for All Blocks
    p_des       = get_param(b_des, 'PortHandles');
    p_sum_err   = get_param(b_sum_err, 'PortHandles');
    p_err_rate  = get_param(b_err_rate, 'PortHandles');
    p_mux1      = get_param(b_mux1, 'PortHandles');
    p_flc       = get_param(b_flc, 'PortHandles');
    p_calc_omega= get_param(b_calc_omega, 'PortHandles');
    p_wr        = get_param(b_wr, 'PortHandles');
    p_sc_slip   = get_param(b_sc_slip, 'PortHandles');
    p_fl        = get_param(b_fl, 'PortHandles');
    p_sp_der    = get_param(b_sp_der, 'PortHandles');
    p_fr        = get_param(b_fr, 'PortHandles');
    p_sum_force = get_param(b_sum_force, 'PortHandles');
    p_inv_m     = get_param(b_inv_m, 'PortHandles');
    p_int_v     = get_param(b_int_v, 'PortHandles');
    p_stop      = get_param(b_stop, 'PortHandles');
    p_sp_sq     = get_param(b_sp_sq, 'PortHandles');
    p_fa        = get_param(b_fa, 'PortHandles');
    p_lut       = get_param(b_lut, 'PortHandles');
    p_prod      = get_param(b_prod, 'PortHandles');
    p_fz        = get_param(b_fz, 'PortHandles');
    p_sum_fx    = get_param(b_sum_fx, 'PortHandles');
    p_wr1       = get_param(b_wr1, 'PortHandles');
    p_calc_slip = get_param(b_calc_slip, 'PortHandles');
    p_int_dist  = get_param(b_int_dist, 'PortHandles');
    p_mux2      = get_param(b_mux2, 'PortHandles');
    p_sc_vel    = get_param(b_sc_vel, 'PortHandles');
    p_sc_dist   = get_param(b_sc_dist, 'PortHandles');

    %% Configure Signal Logging on Outports
    set_param(p_calc_slip.Outport(1),  'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'slip');
    set_param(p_calc_omega.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'wheel_speed');
    set_param(p_int_v.Outport(1),      'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'velocity');
    set_param(p_int_dist.Outport(1),   'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'stopping_distance');
    set_param(p_sum_fx.Outport(1),     'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Fx');
    set_param(p_inv_m.Outport(1),      'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'v_dot');

    %% Connections and Line Routing Matching Fig. 7
    % 1. Desired slip -> Sum_error port 1
    add_line(model_name, p_des.Outport(1), p_sum_err.Inport(1), 'autorouting', 'smart');

    % 2. Sum_error -> Mux1 port 1 and Error rate port 1
    l_err = add_line(model_name, p_sum_err.Outport(1), p_mux1.Inport(1), 'autorouting', 'smart');
    set_param(l_err, 'Name', 'Error');
    add_line(model_name, p_sum_err.Outport(1), p_err_rate.Inport(1), 'autorouting', 'smart');

    % 3. Error rate -> Mux1 port 2
    l_err_rate = add_line(model_name, p_err_rate.Outport(1), p_mux1.Inport(2), 'autorouting', 'smart');
    set_param(l_err_rate, 'Name', 'Error rate');

    % 4. Mux1 -> Fuzzy Logic Controller
    add_line(model_name, p_mux1.Outport(1), p_flc.Inport(1), 'autorouting', 'smart');

    % 5. Fuzzy Logic Controller -> Calculating wheel speed port 1
    l_dp = add_line(model_name, p_flc.Outport(1), p_calc_omega.Inport(1), 'autorouting', 'smart');
    set_param(l_dp, 'Name', 'Change in pressure');

    % 6. Calculating wheel speed -> Mux2 port 1 and Wheel radius1 port 1
    add_line(model_name, p_calc_omega.Outport(1), p_mux2.Inport(1), 'autorouting', 'smart');
    add_line(model_name, p_calc_omega.Outport(1), p_wr1.Inport(1), 'autorouting', 'smart');

    % 7. Wheel radius1 -> Actural slip port 1 (Wheel speed input)
    add_line(model_name, p_wr1.Outport(1), p_calc_slip.Inport(1), 'autorouting', 'smart');

    % 8. Actural slip -> Sum_error port 2, Slip scope, mu vs slip lookup table
    add_line(model_name, p_calc_slip.Outport(1), p_sum_err.Inport(2), 'autorouting', 'smart');
    add_line(model_name, p_calc_slip.Outport(1), p_sc_slip.Inport(1), 'autorouting', 'smart');
    add_line(model_name, p_calc_slip.Outport(1), p_lut.Inport(1), 'autorouting', 'smart');

    % 9. mu vs slip lookup table -> Product port 2 and Fz port 1
    add_line(model_name, p_lut.Outport(1), p_prod.Inport(2), 'autorouting', 'smart');
    add_line(model_name, p_lut.Outport(1), p_fz.Inport(1), 'autorouting', 'smart');

    % 10. Speed derivative -> F-load transfer port 1
    add_line(model_name, p_sp_der.Outport(1), p_fl.Inport(1), 'autorouting', 'smart');

    % 11. F-load transfer -> Product port 1
    add_line(model_name, p_fl.Outport(1), p_prod.Inport(1), 'autorouting', 'smart');

    % 12. Product -> Sum_Fx port 1
    add_line(model_name, p_prod.Outport(1), p_sum_fx.Inport(1), 'autorouting', 'smart');

    % 13. Fz -> Sum_Fx port 2
    add_line(model_name, p_fz.Outport(1), p_sum_fx.Inport(2), 'autorouting', 'smart');

    % 14. Sum_Fx -> Sum_force port 2 (middle input) and Wheel radius port 1
    l_fx = add_line(model_name, p_sum_fx.Outport(1), p_sum_force.Inport(2), 'autorouting', 'smart');
    set_param(l_fx, 'Name', 'Fx');
    add_line(model_name, p_sum_fx.Outport(1), p_wr.Inport(1), 'autorouting', 'smart');

    % 15. Wheel radius -> Calculating wheel speed port 2 (Tyre torque input)
    l_tt = add_line(model_name, p_wr.Outport(1), p_calc_omega.Inport(2), 'autorouting', 'smart');
    set_param(l_tt, 'Name', 'Tyre torque');

    % 16. Fr -> Sum_force port 1 (top input)
    add_line(model_name, p_fr.Outport(1), p_sum_force.Inport(1), 'autorouting', 'smart');

    % 17. Speed Square -> Fa port 1
    add_line(model_name, p_sp_sq.Outport(1), p_fa.Inport(1), 'autorouting', 'smart');

    % 18. Fa -> Sum_force port 3 (bottom input)
    add_line(model_name, p_fa.Outport(1), p_sum_force.Inport(3), 'autorouting', 'smart');

    % 19. Sum_force -> (1/m) port 1
    add_line(model_name, p_sum_force.Outport(1), p_inv_m.Inport(1), 'autorouting', 'smart');

    % 20. (1/m) -> Integrator_v port 1
    add_line(model_name, p_inv_m.Outport(1), p_int_v.Inport(1), 'autorouting', 'smart');

    % 21. Integrator_v port 1 (velocity v) branches:
    add_line(model_name, p_int_v.Outport(1), p_sp_der.Inport(1), 'autorouting', 'smart');
    add_line(model_name, p_int_v.Outport(1), p_mux2.Inport(2), 'autorouting', 'smart');
    add_line(model_name, p_int_v.Outport(1), p_sp_sq.Inport(1), 'autorouting', 'smart');
    add_line(model_name, p_int_v.Outport(1), p_calc_slip.Inport(2), 'autorouting', 'smart');
    add_line(model_name, p_int_v.Outport(1), p_int_dist.Inport(1), 'autorouting', 'smart');

    % 22. Integrator_v port 2 (saturation port) -> STOP port 1
    add_line(model_name, p_int_v.Outport(2), p_stop.Inport(1), 'autorouting', 'smart');

    % 23. Mux2 -> Velocity scope
    add_line(model_name, p_mux2.Outport(1), p_sc_vel.Inport(1), 'autorouting', 'smart');

    % 24. Integrator_dist -> Stopping distance scope
    add_line(model_name, p_int_dist.Outport(1), p_sc_dist.Inport(1), 'autorouting', 'smart');

    % Save model directly to the project directory
    target_file = fullfile(model_dir, [model_name '.slx']);
    save_system(model_name, target_file);
    fprintf('Successfully built and saved %s\n', target_file);
end

%% Helper: Build Subsystem 'Calculating wheel speed'
function build_subsystem_wheel_speed(sys_path)
    Simulink.SubSystem.deleteContents(sys_path);

    % Inport 1: Change in pressure (dP)
    b_in_dp = add_block('simulink/Sources/In1', [sys_path '/Change in pressure'], ...
        'Position', [50, 78, 80, 92]);

    % Integrator: Pressure Integrator (0 to P_max)
    b_int_p = add_block('simulink/Continuous/Integrator', [sys_path '/Integrator'], ...
        'InitialCondition', '0', ...
        'LimitOutput', 'on', ...
        'LowerSaturationLimit', 'P_min', ...
        'UpperSaturationLimit', 'P_max', ...
        'Position', [130, 70, 160, 100]);

    % Gain: Kb (Pressure to Torque)
    b_kb = add_block('simulink/Math Operations/Gain', [sys_path '/Kb'], ...
        'Gain', 'Kb', ...
        'Position', [200, 70, 240, 100]);

    % Inport 2: Tyre torque (Tt)
    b_in_tt = add_block('simulink/Sources/In1', [sys_path '/Tyre torque'], ...
        'Position', [50, 148, 80, 162]);

    % Sum: Tt - Tb
    b_sum_t = add_block('simulink/Math Operations/Sum', [sys_path '/Sum'], ...
        'IconShape', 'round', ...
        'ListOfSigns', '+-|', ...
        'Position', [280, 140, 310, 170]);

    % Gain: 1/Iw
    b_inv_iw = add_block('simulink/Math Operations/Gain', [sys_path '/1//Iw'], ...
        'Gain', '1/Iw', ...
        'Position', [350, 140, 390, 170]);

    % Integrator: Wheel Speed (omega >= 0)
    b_int_om = add_block('simulink/Continuous/Integrator', [sys_path '/Integrator1'], ...
        'InitialCondition', 'omega0', ...
        'LimitOutput', 'on', ...
        'LowerSaturationLimit', '0', ...
        'UpperSaturationLimit', 'inf', ...
        'Position', [430, 140, 460, 170]);

    % Outport 1: Wheel speed
    b_out_om = add_block('simulink/Sinks/Out1', [sys_path '/Wheel speed'], ...
        'Position', [500, 148, 530, 162]);

    % Connect subsystem ports
    p_in_dp  = get_param(b_in_dp, 'PortHandles');
    p_int_p  = get_param(b_int_p, 'PortHandles');
    p_kb     = get_param(b_kb, 'PortHandles');
    p_in_tt  = get_param(b_in_tt, 'PortHandles');
    p_sum_t  = get_param(b_sum_t, 'PortHandles');
    p_inv_iw = get_param(b_inv_iw, 'PortHandles');
    p_int_om = get_param(b_int_om, 'PortHandles');
    p_out_om = get_param(b_out_om, 'PortHandles');

    add_line(sys_path, p_in_dp.Outport(1), p_int_p.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_int_p.Outport(1), p_kb.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_kb.Outport(1), p_sum_t.Inport(2), 'autorouting', 'smart');
    add_line(sys_path, p_in_tt.Outport(1), p_sum_t.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_sum_t.Outport(1), p_inv_iw.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_inv_iw.Outport(1), p_int_om.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_int_om.Outport(1), p_out_om.Inport(1), 'autorouting', 'smart');
end

%% Helper: Build Subsystem 'Actural slip' (Actual relative slip)
function build_subsystem_slip(sys_path)
    Simulink.SubSystem.deleteContents(sys_path);

    % Inport 1: Wheel speed (omega * R)
    b_in_w = add_block('simulink/Sources/In1', [sys_path '/Wheel speed'], ...
        'Position', [50, 78, 80, 92]);

    % Inport 2: vehicle speed (v)
    b_in_v = add_block('simulink/Sources/In1', [sys_path '/vehicle speed'], ...
        'Position', [50, 148, 80, 162]);

    % Sum: v - omega*R (Numerator)
    b_sum_dv = add_block('simulink/Math Operations/Sum', [sys_path '/Sum'], ...
        'IconShape', 'rectangular', ...
        'ListOfSigns', '+-', ...
        'Position', [140, 105, 170, 135]);

    % Constant: eps (Denominator safety)
    b_eps = add_block('simulink/Sources/Constant', [sys_path '/eps'], ...
        'Value', '1e-5', ...
        'Position', [140, 200, 170, 220]);

    % Max Block: max(v, eps) (Denominator)
    b_max_v = add_block('simulink/Math Operations/MinMax', [sys_path '/Max'], ...
        'Function', 'max', ...
        'Inputs', '2', ...
        'Position', [220, 160, 250, 200]);

    % Divide: (v - omega*R) / max(v, eps)
    b_div = add_block('simulink/Math Operations/Divide', [sys_path '/Divide'], ...
        'Inputs', '*/', ...
        'Position', [300, 115, 330, 155]);

    % Outport 1: Actual relative slip (lambda)
    b_out_s = add_block('simulink/Sinks/Out1', [sys_path '/Actual relative slip'], ...
        'Position', [380, 128, 410, 142]);

    % Connect subsystem ports
    p_in_w  = get_param(b_in_w, 'PortHandles');
    p_in_v  = get_param(b_in_v, 'PortHandles');
    p_sum_dv= get_param(b_sum_dv, 'PortHandles');
    p_eps   = get_param(b_eps, 'PortHandles');
    p_max_v = get_param(b_max_v, 'PortHandles');
    p_div   = get_param(b_div, 'PortHandles');
    p_out_s = get_param(b_out_s, 'PortHandles');

    add_line(sys_path, p_in_v.Outport(1), p_sum_dv.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_in_w.Outport(1), p_sum_dv.Inport(2), 'autorouting', 'smart');
    add_line(sys_path, p_in_v.Outport(1), p_max_v.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_eps.Outport(1), p_max_v.Inport(2), 'autorouting', 'smart');
    add_line(sys_path, p_sum_dv.Outport(1), p_div.Inport(1), 'autorouting', 'smart');
    add_line(sys_path, p_max_v.Outport(1), p_div.Inport(2), 'autorouting', 'smart');
    add_line(sys_path, p_div.Outport(1), p_out_s.Inport(1), 'autorouting', 'smart');
end
