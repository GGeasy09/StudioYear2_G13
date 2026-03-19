%% Basic Setup
clear; clc; close all;
% Load your base parameters
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.4626;
ki_value = 0.004608;
kd_value = 0.18;
rad2deg = 180/pi;
Setpoint = 180;
Initial_point = 0;
T_Kd_Filter = 0.01;

% For parsim, just use the model name
simfile = 'Noise.slx'; 
load_system("Noise.slx");

% Define your methods and noise
Control_Method = [1,2,3,4,5,6,7,8,9]; 
Noise_Power = 0.00001;
Noise_Frequencys = [0.0001, 0.001, 0.01];

% --- NEW: Descriptive Names for Titles ---
% This maps directly to methods 1 through 9 based on your rules
method_names = {
    'Discrete Model with 1000Hz Lowpass Filter';       % Method 1
    'Continuous Model with 1000Hz Lowpass Filter';     % Method 2
    'Constant Block Model with 1000Hz Lowpass Filter'; % Method 3
    'Discrete Model without Filter';                   % Method 4
    'Continuous Model without Filter';                 % Method 5
    'Constant Block Model without Filter';             % Method 6
    'Discrete Model with 10000Hz Lowpass Filter';      % Method 7
    'Continuous Model with 10000Hz Lowpass Filter';    % Method 8
    'Constant Block Model with 10000Hz Lowpass Filter' % Method 9
};

%% Pre-allocation for Table Data
num_methods = length(Control_Method);
num_freqs = length(Noise_Frequencys);
total_runs = num_methods * num_freqs;

tbl_Method_ID = zeros(total_runs, 1);
tbl_Noise_Freq = zeros(total_runs, 1);
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
    
    % Update Figure Name
    fig = figure('Name', current_name, 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.6 0.8]);
    
    % Prepare Subplots with Updated Titles
    ax_pos = subplot(2, 1, 1); hold on; grid on;
    xlabel('Time (s)'); ylabel('Position (Degrees)');
    title(['Position Response: ', current_name]); % NEW descriptive title
    
    ax_effort = subplot(2, 1, 2); hold on; grid on;
    xlabel('Time (s)'); ylabel('Control Effort (Volts)');
    title(['Control Effort: ', current_name]);    % NEW descriptive title
    
    freq_legends = cell(num_freqs, 1);
    
    for j = 1:num_freqs
        % Update Noise Frequency for Simulink
        Noise_Frequency = Noise_Frequencys(j); 
        
        % Run Simulation
        simout = sim(simfile);
        
        % Extract Data
        data_signal = simout.Position_Data.Data * rad2deg;
        time_signal = simout.Position_Data.Time;
        effort_signal = simout.control_effort.Data; 
        
        % Plot Current Frequency Data
        plot(ax_pos, time_signal, data_signal, 'LineWidth', 1.5);
        plot(ax_effort, time_signal, effort_signal, 'LineWidth', 1.5);
        freq_legends{j} = sprintf('Noise Freq = %g Hz', Noise_Frequency);
        
        % Calculate Metrics
        [max_val, max_idx] = max(data_signal);
        current_peak_time = time_signal(max_idx);
        
        os_calc = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
        current_os = max(0, os_calc); 
        
        t10_idx = find(data_signal >= 0.1 * Setpoint, 1);
        t90_idx = find(data_signal >= 0.9 * Setpoint, 1);
        if ~isempty(t10_idx) && ~isempty(t90_idx)
            current_rise = time_signal(t90_idx) - time_signal(t10_idx);
        else
            current_rise = NaN; 
        end
        
        error = abs(data_signal - Setpoint);
        tolerance = 0.02 * abs(Setpoint - Initial_point);
        last_outside_idx = find(error > tolerance, 1, 'last');
        
        if isempty(last_outside_idx)
            current_settle = 0; 
        elseif last_outside_idx == length(time_signal)
            current_settle = NaN; 
        else
            current_settle = time_signal(last_outside_idx + 1);
        end
        
        % Store in Table Arrays
        tbl_Method_ID(run_idx) = Controller_Method;
        tbl_Noise_Freq(run_idx) = Noise_Frequency;
        peak_times(run_idx) = current_peak_time;
        overshoots(run_idx) = current_os;
        rise_times(run_idx) = current_rise;
        settling_times(run_idx) = current_settle;
        
        run_idx = run_idx + 1;
    end
    
    % Finalize Formatting for Current Figure
    yline(ax_pos, Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    legend(ax_pos, freq_legends, 'Location', 'southeast');
    
    yline(ax_effort, 12, 'r--', 'Max Voltage (+12V)', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    yline(ax_effort, -12, 'r--', 'Min Voltage (-12V)', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    legend(ax_effort, freq_legends, 'Location', 'northeast');
end

%% --- Display Combined Results Table ---
ResultsTable = table(tbl_Method_ID, tbl_Noise_Freq, peak_times, overshoots, rise_times, settling_times, ...
    'VariableNames', {'Method_ID', 'Noise_Frequency_Hz', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct'});
disp('--- Simulation Results across Methods and Noise Frequencies ---');
disp(ResultsTable);