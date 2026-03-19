%% Basic Setup
clear; 
% Make sure this path is correct for your machine
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.4626;
ki_value = 0.004608;
kd_value = 0.18;
rad2deg = 180/pi;
Setpoint = 180 + 360*10;
Initial_point = 0;

Method_name = ["Discerate Model with Aniti Windup","Continuous Model with Aniti Windup",...
               "Constant block with Aniti Windup","Discerate Model without Aniti Windup",...
               "Continuous Model without Aniti Windup","Constant block without Aniti Windup"];
           
simfile = 'Anti_Windup'; % Dropped the .slx, sim() prefers just the model name
load_system(simfile);

% Define the test matrix
Control_Methods = [1,4];  % e.g., PI and PID with Anti-Windup
Saturation_States = [1,2];     % 1 = No Saturation, 2 = Saturation Enabled

%% Pre-allocation
total_runs = length(Control_Methods) * length(Saturation_States);
results_pos = cell(total_runs, 1);
results_time = cell(total_runs, 1);
results_effort = cell(total_runs, 1);
legend_labels = cell(total_runs, 1);

% Metrics
peak_times = zeros(total_runs, 1);
overshoots = zeros(total_runs, 1);
rise_times = zeros(total_runs, 1);
settling_times = zeros(total_runs, 1);
ss_errors = zeros(total_runs, 1);

%% --- Nested Simulation Loop (UPGRADED) ---
run_idx = 1;
for m = 1:length(Control_Methods)
    for s = 1:length(Saturation_States)
        
        % 1. Create a SimulationInput object for clean execution
        simIn = Simulink.SimulationInput(simfile);
        
        % 2. Safely inject variables directly into the model for this specific run
        simIn = simIn.setVariable('Controller_Method', Control_Methods(m));
        simIn = simIn.setVariable('Saturation', Saturation_States(s));
        
        % 3. Run Simulation (Outputs are neatly packaged into simout)
        simout = sim(simIn);
        
        % 4. Extract Data 
        % (Assuming 'Position_Data' and 'control_effort' are "To Workspace" blocks)
        data_signal = simout.Position_Data.Data * rad2deg;
        time_signal = simout.Position_Data.Time;
        effort_signal = simout.control_effort.Data; 
        
        % Store Raw Data
        results_pos{run_idx} = data_signal;
        results_time{run_idx} = time_signal;
        results_effort{run_idx} = effort_signal;
        
        % --- Calculate Metrics ---
        
        % 1. Steady State Error (Final value comparison)
        ss_errors(run_idx) = abs(Setpoint - data_signal(end));
        
        % 2. Peak & Overshoot
        [max_val, max_peak_idx] = max(data_signal);
        peak_times(run_idx) = time_signal(max_peak_idx);
        os_calc = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
        overshoots(run_idx) = max(0, os_calc); 
        
        % 3. Rise Time (10% to 90%)
        t10_idx = find(data_signal >= 0.1 * Setpoint, 1);
        t90_idx = find(data_signal >= 0.9 * Setpoint, 1);
        if ~isempty(t10_idx) && ~isempty(t90_idx)
            rise_times(run_idx) = time_signal(t90_idx) - time_signal(t10_idx);
        else
            rise_times(run_idx) = NaN;
        end
        
        % 4. Settling Time (2% Criterion)
        error_signal = abs(data_signal - Setpoint);
        tolerance = 0.02 * abs(Setpoint - Initial_point);
        last_out = find(error_signal > tolerance, 1, 'last');
        if isempty(last_out)
            settling_times(run_idx) = 0;
        elseif last_out == length(time_signal)
            settling_times(run_idx) = NaN;
        else
            settling_times(run_idx) = time_signal(last_out + 1);
        end
        
        % Create Label
        sat_text = "No Sat"; 
        if Saturation_States(s) == 2, sat_text = "Sat"; end
        
        method_str = Method_name(Control_Methods(m));
        legend_labels{run_idx} = sprintf('%s (%s)', method_str, sat_text);
        
        run_idx = run_idx + 1;
    end
end

%% --- Display Results Table ---
ResultsTable = table(legend_labels, peak_times, overshoots, rise_times, settling_times, ss_errors, ...
    'VariableNames', {'Configuration', 'PeakTime_s', 'Overshoot_pct', 'RiseTime_s', 'SettlingTime_s', 'SSError_deg'});
disp(ResultsTable);

%% --- Plotting ---
% Figure 1: Position
figure('Name', 'Response Comparison', 'Color', 'w');
hold on;
for i = 1:total_runs
    plot(results_time{i}, results_pos{i}, 'LineWidth', 1.5);
end
yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.5);
grid on; 
xlabel('Time (s)'); 
ylabel('Position (deg)');
title('System Response: Position vs. Time'); 
legend(legend_labels, 'Location', 'best');

% Figure 2: Effort
figure('Name', 'Effort Comparison', 'Color', 'w');
hold on;
for i = 1:total_runs
    plot(results_time{i}, results_effort{i}, 'LineWidth', 1.5);
end
yline([12, -12], 'r--', 'Saturation Limit', 'LineWidth', 1.5);
grid on; 
xlabel('Time (s)'); 
ylabel('Voltage (V)');
title('Controller Effort vs. Time'); 
legend(legend_labels, 'Location', 'best');