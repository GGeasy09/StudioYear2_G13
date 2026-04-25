%% Basic Setup
clear; clc;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section (Vectorized)
%block
kp_value = 1.9642;
ki_value = 0;
kd_value = 1.8856;


%% Input Variable Section
Initial_point = 0; 
Setpoint = 90; 
rad2deg = 180/pi;
simfile = "C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Part3_PID_Design\Tuning&Validation\Validation.slx";

% Define your methods here (e.g., 1 = P, 2 = PI, 3 = PID)
Control_Method = [1, 2, 3]; 

% Pre-allocate based on the number of methods
num_methods = length(Control_Method);
results_pos = cell(num_methods, 1);
results_time = cell(num_methods, 1);
peak_times = zeros(num_methods, 1);
overshoots = zeros(num_methods, 1);
legend_labels = cell(num_methods, 1);

%% --- Simulation Loop ---
for i = 1:num_methods
    % Update the Control_Method variable for Simulink
    Controller_Method = Control_Method(i); 
    
    % Run Simulation
    simout = sim(simfile);
    
    % Extract Data
    data_signal = simout.Position_Data.Data * rad2deg;
    time_signal = simout.Position_Data.Time;
    
    % Store for plotting
    results_pos{i} = data_signal;
    results_time{i} = time_signal;
    
    % --- Calculate Metrics ---
    [max_val, max_idx] = max(data_signal);
    peak_times(i) = time_signal(max_idx);
    
    % Percentage Overshoot 
    os_calc = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
    overshoots(i) = max(0, os_calc); 

    % --- Calculate Additional Metrics ---
    
    % 1. Rise Time (10% to 90%)
    t10_idx = find(data_signal >= 0.1 * Setpoint, 1);
    t90_idx = find(data_signal >= 0.9 * Setpoint, 1);
    if ~isempty(t10_idx) && ~isempty(t90_idx)
        rise_times(i) = time_signal(t90_idx) - time_signal(t10_idx);
    else
        rise_times(i) = NaN; % If it never reaches 90%
    end

    % 2. Settling Time (2% Criterion)
    % We work backwards from the end to find the last time it was outside the 2% bound
    error = abs(data_signal - Setpoint);
    tolerance = 0.02 * abs(Setpoint - Initial_point);
    last_outside_idx = find(error > tolerance, 1, 'last');
    
    if isempty(last_outside_idx)
        settling_times(i) = 0; 
    elseif last_outside_idx == length(time_signal)
        settling_times(i) = NaN; % System hasn't settled yet
    else
        settling_times(i) = time_signal(last_outside_idx + 1);
    end
    
    % Create Legend Label
    legend_labels{i} = sprintf('Method %d', Controller_Method);
end

%% --- Display Results Table ---
% Add these to your pre-allocation section
rise_times = zeros(num_methods, 1);
settling_times = zeros(num_methods, 1);

% Update your table display
ResultsTable = table(Control_Method', peak_times, overshoots, rise_times, settling_times, ...
    'VariableNames', {'Method_ID', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct'});
disp(ResultsTable);

%% --- Figure: Combined Comparison ---
figure('Name', 'Control Method Comparison', 'Color', 'w');
hold on;
for i = 1:num_methods
    plot(results_time{i}, results_pos{i}, 'LineWidth', 1.5);
end

yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Position (Degrees)');
title('System Response: Comparison of Different Control Methods');
legend(legend_labels, 'Location', 'southeast');
grid on;