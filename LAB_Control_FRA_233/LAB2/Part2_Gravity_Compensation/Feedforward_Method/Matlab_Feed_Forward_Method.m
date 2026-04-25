%% Basic Setup
clear;
run("Lab2_params_student.m");

%% Input Variables
kp_value = 0;     
T_vector = [8.947e-04]; 
rad2deg = 180/pi;
simfile = "Feedforward_Method.slx";
Setpoint = 90;
Initial_point = 90;

% Storage
results_pos = {};
results_time = {};
legend_labels = {};
result_control_Effort = {};
sse_values = []; % Array to store SSE for each run

%% --- Run 1: Method 0 (Baseline) ---
Method = 1;
T = 1;
fprintf('--- Results for Method 0 ---\n');
simout0 = sim(simfile);

% Calculate SSE for Method 0
final_val0 = simout0.Position_Data.Data(end) * rad2deg;
sse0 = abs(Setpoint - final_val0);
sse_values(1) = sse0;

results_pos{end+1} = simout0.Position_Data.Data * rad2deg;
results_time{end+1} = simout0.Position_Data.Time;
result_control_Effort{end+1} = simout0.FFD_Effort.Data;
legend_labels{end+1} = sprintf('Method 0 (SSE: %.4f)', sse0);

fprintf('Method 0 SSE: %.4f degrees\n\n', sse0);

%% --- Run 2: Method 1 (Looping through T) ---
Method = 2; 
fprintf('--- Results for Method 1 ---\n');
for i = 1:length(T_vector)
    T = T_vector(i); 
    simout = sim(simfile);
    
    % Calculate SSE for this iteration
    final_val = simout.Position_Data.Data(end) * rad2deg;
    current_sse = abs(Setpoint - final_val);
    sse_values(end+1) = current_sse;
    
    % Store data
    results_pos{end+1} = simout.Position_Data.Data * rad2deg;
    results_time{end+1} = simout.Position_Data.Time;
    legend_labels{end+1} = sprintf('T=%g (SSE: %.4f)', T, current_sse);
    
    result_control_Effort{end+1} = simout.FFD_Effort.Data;

    fprintf('T = %g | SSE: %.4f degrees\n', T, current_sse);
end

%% --- Figure: Combined Comparison ---
figure('Name', 'Steady State Error Analysis');
hold on;

% Plot each simulation run
for j = 1:length(results_pos)
    if j == 1
        plot(results_time{j}, results_pos{j}, 'k--', 'LineWidth', 2);
    else
        plot(results_time{j}, results_pos{j}, 'LineWidth', 1.2);
    end
end

% Plot the Setpoint Line
h_sp = yline(Setpoint, 'r-', 'Setpoint', 'LineWidth', 2);
h_sp.LabelVerticalAlignment = 'bottom'; % Moves label above line

xlabel('Time (s)');
ylabel('Position (Degrees)');
title('System Response and Steady-State Error');
legend(legend_labels, 'Location', 'southeast');
grid on;

%% --- Figure: Control Effort Analysis ---
figure('Name', 'Control Effort Comparison');
hold on;

% Loop through and plot the control effort stored in your cell array
for k = 1:length(result_control_Effort)
    % Find the time vector associated with this run
    % (Using results_time which matches indices with result_control_Effort)
    if k == 1
        plot(results_time{k}, result_control_Effort{k}, 'k--', 'LineWidth', 2);
    else
        plot(results_time{k}, result_control_Effort{k}, 'LineWidth', 1.2);
    end
end

xlabel('Time (s)');
ylabel('Control Effort (e.g., Volts or PWM)');
title('Controller Output Over Time');
legend(legend_labels, 'Location', 'northeast'); % Reuses your labels
grid on;