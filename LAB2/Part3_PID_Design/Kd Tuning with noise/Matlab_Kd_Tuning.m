%% Basic Setup
clear;
% Load your base parameters
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.4626;
ki_values = 0.004608;
kd_value = [0.008,0.08,0.18,0.28,0.68]; % This will be our inner loop
rad2deg = 180/pi;
Setpoint = 180;
Initial_point = 0;
T_Kd_Filter = 0.01;

% For parsim, just use the model name
simfile = 'Ki_Tuning.slx'; 
load_system(simfile);

% Define your methods and noise
Control_Method = [2]; 
Noise_Power = 0.0001;
Noise_Frequency = 0.001; 

% Descriptive Names for Titles
method_names = {
    'Discrete Model with 100Hz Lowpass Filter';       % Method 1
    'Continuous Model with 100Hz Lowpass Filter';     % Method 2
    'Constant Block Model with 100Hz Lowpass Filter'; % Method 3
};

%% Pre-allocation for Table Data
num_methods = length(Control_Method);
num_kds = length(kd_values);
total_runs = num_methods * num_kds;

tbl_Method_ID = zeros(total_runs, 1);
tbl_Kd_Value = zeros(total_runs, 1); 
peak_times = zeros(total_runs, 1);
overshoots = zeros(total_runs, 1);
rise_times = zeros(total_runs, 1);
settling_times = zeros(total_runs, 1);
r_squared_noise = zeros(total_runs, 1); 
run_idx = 1;

%% --- Simulation Nested Loop ---
for i = 1:num_methods
    Controller_Method = Control_Method(i); 
    
    % Get the descriptive name for the current method
    current_name = method_names{Controller_Method};
    
    % --- Create SUMMARY FIGURE (All Clean Kds combined) ---
    fig_summary = figure('Name', ['Summary (Clean): ', current_name], 'Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.15 0.6 0.8]);
    
    ax_pos_sum = subplot(2, 1, 1); hold on; grid on;
    xlabel('Time (s)'); ylabel('Position (Degrees)');
    title(['Position Response Summary (No Noise): ', current_name]); 
    
    ax_eff_sum = subplot(2, 1, 2); hold on; grid on;
    xlabel('Time (s)'); ylabel('Control Effort (Volts)');
    title(['Control Effort Summary (No Noise): ', current_name]);  
    
    summary_legends = cell(num_kds, 1); % Legend array for the summary graph
    
    for j = 1:num_kds
        % Update Kd value for Simulink
        kd_value = kd_values(j); 
        
        % --- 1. RUN WITHOUT NOISE (CLEAN BASELINE) ---
        Noise_switch = 0;
        assignin('base', 'Noise_switch', Noise_switch); 
        simout_clean = sim(simfile);
        
        pos_clean = simout_clean.Position_Data.Data * rad2deg;
        time_clean = simout_clean.Position_Data.Time;
        effort_clean = simout_clean.control_effort.Data; 
        
        % --- 2. RUN WITH NOISE ---
        Noise_switch = 1;
        assignin('base', 'Noise_switch', Noise_switch);
        simout_noise = sim(simfile);
        
        pos_noise = simout_noise.Position_Data.Data * rad2deg;
        time_noise = simout_noise.Position_Data.Time;
        
        % --- Plot onto SUMMARY FIGURE ---
        % Plotting the clean data on the combined graph
        plot(ax_pos_sum, time_clean, pos_clean, 'LineWidth', 1.5);
        plot(ax_eff_sum, time_clean, effort_clean, 'LineWidth', 1.5);
        summary_legends{j} = sprintf('Kd = %g', kd_value);
        
        % --- Create INDIVIDUAL FIGURE ---
        fig_title = sprintf('%s | Kd = %g', current_name, kd_value);
        fig_indiv = figure('Name', fig_title, 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.6 0.8]);
        
        ax_pos_indiv = subplot(2, 1, 1); hold on; grid on;
        xlabel('Time (s)'); ylabel('Position (Degrees)');
        title(['Position Response (Clean vs Noisy): ', fig_title]); 
        
        ax_eff_indiv = subplot(2, 1, 2); hold on; grid on;
        xlabel('Time (s)'); ylabel('Control Effort (Volts)');
        title(['Control Effort: ', fig_title]);   
        
        % --- Interpolate and Calculate R^2 ---
        pos_noise_interp = interp1(time_noise, pos_noise, time_clean, 'linear', 'extrap');
        SS_res = sum((pos_clean - pos_noise_interp).^2);
        SS_tot = sum((pos_clean - mean(pos_clean)).^2);
        current_r_squared = 1 - (SS_res / SS_tot);
        
        % --- Plot onto INDIVIDUAL FIGURE ---
        p = plot(ax_pos_indiv, time_clean, pos_clean, 'LineWidth', 1.5, 'DisplayName', 'Clean Signal');
        color_used = p.Color; 
        plot(ax_pos_indiv, time_clean, pos_noise_interp, '--', 'Color', [color_used 0.5], 'LineWidth', 1.5, 'DisplayName', 'Noisy Signal');
        plot(ax_eff_indiv, time_clean, effort_clean, 'LineWidth', 1.5, 'Color', color_used, 'DisplayName', 'Control Effort');
        
        % --- Calculate Metrics (Using Clean Signal) ---
        [max_val, max_idx] = max(pos_clean);
        current_peak_time = time_clean(max_idx);
        
        os_calc = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
        current_os = max(0, os_calc); 
        
        t10_idx = find(pos_clean >= 0.1 * Setpoint, 1);
        t90_idx = find(pos_clean >= 0.9 * Setpoint, 1);
        if ~isempty(t10_idx) && ~isempty(t90_idx)
            current_rise = time_clean(t90_idx) - time_clean(t10_idx);
        else
            current_rise = NaN; 
        end
        
        error = abs(pos_clean - Setpoint);
        tolerance = 0.02 * abs(Setpoint - Initial_point);
        last_outside_idx = find(error > tolerance, 1, 'last');
        
        if isempty(last_outside_idx)
            current_settle = 0; 
        elseif last_outside_idx == length(time_clean)
            current_settle = NaN; 
        else
            current_settle = time_clean(last_outside_idx + 1);
        end
        
        % --- Store in Table Arrays ---
        tbl_Method_ID(run_idx) = Controller_Method;
        tbl_Kd_Value(run_idx) = kd_value;
        peak_times(run_idx) = current_peak_time;
        overshoots(run_idx) = current_os;
        rise_times(run_idx) = current_rise;
        settling_times(run_idx) = current_settle;
        r_squared_noise(run_idx) = current_r_squared; 
        
        run_idx = run_idx + 1;
        
        % --- Finalize INDIVIDUAL FIGURE ---
        yline(ax_pos_indiv, Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(ax_pos_indiv, 'Location', 'southeast');
        yline(ax_eff_indiv, 12, 'r--', 'Max Voltage (+12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        yline(ax_eff_indiv, -12, 'r--', 'Min Voltage (-12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(ax_eff_indiv, 'Location', 'northeast');
    end
    
    % --- Finalize SUMMARY FIGURE ---
    yline(ax_pos_sum, Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    legend(ax_pos_sum, summary_legends, 'Location', 'southeast');
    
    yline(ax_eff_sum, 12, 'r--', 'Max Voltage (+12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    yline(ax_eff_sum, -12, 'r--', 'Min Voltage (-12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    legend(ax_eff_sum, summary_legends, 'Location', 'northeast');
end

%% --- Display Combined Results Table ---
ResultsTable = table(tbl_Method_ID, tbl_Kd_Value, peak_times, overshoots, rise_times, settling_times, r_squared_noise, ...
    'VariableNames', {'Method_ID', 'Kd_Value', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct', 'R2_Noise_Robustness'});
disp('--- Simulation Results across Methods and Kd Values ---');
disp(ResultsTable);