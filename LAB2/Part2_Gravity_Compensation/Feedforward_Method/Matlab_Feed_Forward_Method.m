%% Basic Setup
clear;
run("Lab2_params_student.m");

%% Input Variables
kp_value = 0;     
T_vector = [1,0.1,0.01,0.001,0.0001]; 
Setpoint = 270;
rad2deg = 180/pi;
simfile = "Feedforward_Method.slx";
Initial_point = 270;

% Storage
results_pos = {};
results_time = {};
legend_labels = {};
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