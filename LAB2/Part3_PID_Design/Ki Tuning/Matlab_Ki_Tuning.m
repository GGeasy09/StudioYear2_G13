%% Basic Setup
clear;
% Load your base parameters
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.4626;
kd_value = 0.18; % Fixed value
ki_values = [0.0008, 0.002, 0.004608, 0.01, 0.04]; % Array to loop through

rad2deg = 180/pi;
Setpoint = 180;
Initial_point = 0;
T_Kd_Filter = 0.01;

% For parsim, just use the model name
simfile = 'Ki_Tuning.slx'; 
load_system(simfile);

% Define your methods and noise
Control_Method = [2]; 
Noise_switch = 0; % Force noise off for all runs
assignin('base', 'Noise_switch', Noise_switch);

% Descriptive Names for Titles
method_names = {
    'Discrete Model with 100Hz Lowpass Filter';       % Method 1
    'Continuous Model with 100Hz Lowpass Filter';     % Method 2
    'Constant Block Model with 100Hz Lowpass Filter'; % Method 3
};

%% Pre-allocation for Table Data
num_methods = length(Control_Method);
num_kis = length(ki_values);
total_runs = num_methods * num_kis;

tbl_Method_ID = zeros(total_runs, 1);
tbl_Ki_Value = zeros(total_runs, 1); 
peak_times = zeros(total_runs, 1);
overshoots = zeros(total_runs, 1);
rise_times = zeros(total_runs, 1);
settling_times = zeros(total_runs, 1);
run_idx = 1;

%% --- Simulation Nested Loop ---
for i = 1:num_methods
    Controller_Method = Control_Method(i); 
    
    % Get the descriptive name for the current method
    current_name = method_names{Controller_Method};
    
    % --- Create SUMMARY FIGURE (All Clean Kis combined) ---
    fig_summary = figure('Name', ['Summary (Clean): ', current_name], 'Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.15 0.6 0.8]);
    
    ax_pos_sum = subplot(2, 1, 1); hold on; grid on;
    xlabel('Time (s)'); ylabel('Position (Degrees)');
    title(['Position Response Summary: ', current_name]); 
    
    ax_eff_sum = subplot(2, 1, 2); hold on; grid on;
    xlabel('Time (s)'); ylabel('Control Effort (Volts)');
    title(['Control Effort Summary: ', current_name]);  
    
    summary_legends = cell(num_kis, 1); % Legend array for the summary graph
    
    for j = 1:num_kis
        % Update Ki value for Simulink
        ki_value = ki_values(j); 
        assignin('base', 'ki_value', ki_value); % Ensure Simulink sees the updated workspace variable
        
        % --- RUN WITHOUT NOISE (CLEAN BASELINE) ---
        simout_clean = sim(simfile);
        
        pos_clean = simout_clean.Position_Data.Data * rad2deg;
        time_clean = simout_clean.Position_Data.Time;
        effort_clean = simout_clean.control_effort.Data; 
        
        % --- Plot onto SUMMARY FIGURE ONLY ---
        plot(ax_pos_sum, time_clean, pos_clean, 'LineWidth', 1.5);
        plot(ax_eff_sum, time_clean, effort_clean, 'LineWidth', 1.5);
        summary_legends{j} = sprintf('Ki = %g', ki_value);
        
        % --- Calculate Metrics ---
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
        tbl_Ki_Value(run_idx) = ki_value;
        peak_times(run_idx) = current_peak_time;
        overshoots(run_idx) = current_os;
        rise_times(run_idx) = current_rise;
        settling_times(run_idx) = current_settle;
        
        run_idx = run_idx + 1;
    end
    
    % --- Finalize SUMMARY FIGURE ---
    yline(ax_pos_sum, Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    legend(ax_pos_sum, summary_legends, 'Location', 'southeast');
    
    yline(ax_eff_sum, 12, 'r--', 'Max Voltage (+12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    yline(ax_eff_sum, -12, 'r--', 'Min Voltage (-12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    legend(ax_eff_sum, summary_legends, 'Location', 'northeast');
end

%% --- Display Combined Results Table ---
ResultsTable = table(tbl_Method_ID, tbl_Ki_Value, peak_times, overshoots, rise_times, settling_times, ...
    'VariableNames', {'Method_ID', 'Ki_Value', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct'});
disp('--- Simulation Results across Methods and Ki Values ---');
disp(ResultsTable);