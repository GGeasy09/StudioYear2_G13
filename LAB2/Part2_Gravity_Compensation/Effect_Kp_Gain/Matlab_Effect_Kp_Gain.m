%% Basic Setup
clear;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section (Vectorized)
kp_vector = [0.05, 0.0788, 0.12]; % Define the values you want to test
Initial_point = 0; %ใส่ค่าจุดเริ่ม
Setpoint = 90; %ใส่ค่าจุดสุดท้าย
rad2deg = 180/pi;
simfile = "Effect_Kp_Gain.slx";

% Pre-allocate cell arrays to store results
results_pos = cell(length(kp_vector), 1);
results_time = cell(length(kp_vector), 1);
legend_labels = cell(length(kp_vector), 1);

%% --- Simulation Loop with Metrics ---
% Pre-allocate arrays for metrics
peak_times = zeros(length(kp_vector), 1);
overshoots = zeros(length(kp_vector), 1);

for i = 1:length(kp_vector)
    kp_value = kp_vector(i); 
    simout = sim(simfile);
    
    % Extract data
    data_signal = simout.Position_Data.Data * rad2deg;
    time_signal = simout.Position_Data.Time;
    
    % Store for plotting
    results_pos{i} = data_signal;
    results_time{i} = time_signal;
    
    % --- Calculate Metrics ---
    % 1. Find Maximum Value and its index
    [max_val, max_idx] = max(data_signal);
    
    % 2. Peak Time (time at which max value occurs)
    peak_times(i) = time_signal(max_idx);
    
    % 3. Percentage Overshoot 
    % Formula: ((PeakValue - FinalValue) / (FinalValue - InitialValue)) * 100
    overshoots(i) = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
    
    % Ensure overshoot isn't negative (in case it never reaches setpoint)
    if overshoots(i) < 0, overshoots(i) = 0; end
    
    legend_labels{i} = sprintf('k_p = %.4f', kp_value);
end

%% --- Display Results Table ---
ResultsTable = table(kp_vector', peak_times, overshoots, ...
    'VariableNames', {'Kp_Value', 'PeakTime_s', 'PercentOvershoot'});
disp(ResultsTable);

%% --- Figure: Combined Position Comparison ---
figure('Name', 'Multi-Run Position Comparison');
hold on;

for i = 1:length(kp_vector)
    plot(results_time{i}, results_pos{i}, 'LineWidth', 1.5);
end

yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Position (Degrees)');
title('System Response for Multiple k_p Values');
legend(legend_labels);
grid on;