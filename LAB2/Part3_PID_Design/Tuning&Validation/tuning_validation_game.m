%% Basic Setup
clear; clc;
run("C:\Users\USER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% --- Input Variable Section ---
% 1. The Fixed Step Scenario
Initial_point = 0;
Setpoint = 180;
rad2deg = 180/pi;

% 2. The 3 Control Methods (Your Legend Names)
Control_Method = [1, 2, 3];
method_names = {'Tuning Block Discrete', 'Tuning Block Continue', 'Block Constant'};
num_methods = length(Control_Method);

% 3. The 4 PID Parameter Sets
Kp_set = [3.6237,  0.9611,   0.4626,   0.4608  ,  0.75813,  1.0393,   1.9642,  0.18048];
Ki_set = [1.3254,  0.17965,  0.004608, 0.000185,  0,        0,        0,       0      ];
Kd_set = [2.2012,  1.0258,   0.18,     0.18    ,  0.62585,  0.78142,  1.8856,  0.0705 ];
num_pids = length(Kp_set);

model_name = 'Validation'; 
load_system("C:\Users\USER\Documents\GitHub\StudioYear2_G13\LAB2\Part3_PID_Design\Tuning&Validation\Validation.slx");

%% --- STEP 1: Build the Parallel Simulation Batch ---
fprintf('Building parallel simulation inputs...\n');
total_sims = num_pids * num_methods;

% Initialize the array backwards to safely pre-allocate it in memory
for k = total_sims:-1:1
    simInputs(k) = Simulink.SimulationInput(model_name);
end

sim_idx = 1;
for pid_idx = 1:num_pids
    for method_idx = 1:num_methods
        % Assign the fixed step values
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Initial_point', Initial_point);
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Setpoint', Setpoint);
        
        % Assign the specific method (1, 2, or 3)
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Controller_Method', Control_Method(method_idx));
        % Force the simulation into Accelerator mode for speed
        simInputs(sim_idx) = simInputs(sim_idx).setModelParameter('SimulationMode', 'accelerator'); % <--- ADDED THIS LINE
        
        % Assign the specific PID gains for this group
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('kp_value', Kp_set(pid_idx));
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('ki_value', Ki_set(pid_idx));
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('kd_value', Kd_set(pid_idx));
        
        sim_idx = sim_idx + 1;
    end
end

%% --- STEP 2: Run All Simulations Simultaneously ---
fprintf('Running %d simulations in parallel...\n', total_sims);
simOutputs = parsim(simInputs, 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on', 'UseFastRestart', 'on');

%% --- STEP 3: Process Results, Print Tables, and Plot ---
fprintf('\nProcessing results...\n');

sim_idx = 1; 

for pid_idx = 1:num_pids
    
    fprintf('\n======================================================\n');
    fprintf('PID SET %d: Kp = %.4f, Ki = %.6f, Kd = %.4f\n', pid_idx, Kp_set(pid_idx), Ki_set(pid_idx), Kd_set(pid_idx));
    fprintf('======================================================\n');
    
    % Arrays for this specific table (comparing the 3 methods)
    peak_times = zeros(num_methods, 1);
    overshoots = zeros(num_methods, 1);
    rise_times = zeros(num_methods, 1);     
    settling_times = zeros(num_methods, 1); 
    ss_errors = zeros(num_methods, 1);      
    
    fig_name = sprintf('PID Set %d Comparison (Kp=%.2f, Ki=%.4f, Kd=%.2f)', pid_idx, Kp_set(pid_idx), Ki_set(pid_idx), Kd_set(pid_idx));
    figure('Name', fig_name, 'Color', 'w');
    hold on;

    for method_idx = 1:num_methods
        % Extract data from the parallel output object
        data_signal = simOutputs(sim_idx).Position_Data.Data * rad2deg;
        time_signal = simOutputs(sim_idx).Position_Data.Time;
        
        is_step_up = Setpoint > Initial_point;
        
        % --- Calculate Metrics ---
        if is_step_up
            [peak_val, peak_idx] = max(data_signal);
            os_calc = ((peak_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
        else
            [peak_val, peak_idx] = min(data_signal);
            os_calc = ((Setpoint - peak_val) / abs(Setpoint - Initial_point)) * 100;
        end
        
        peak_times(method_idx) = time_signal(peak_idx);
        overshoots(method_idx) = max(0, os_calc);
        
        step_size = Setpoint - Initial_point;
        threshold_10 = Initial_point + 0.1 * step_size;
        threshold_90 = Initial_point + 0.9 * step_size;
        
        if is_step_up
            t10_idx = find(data_signal >= threshold_10, 1);
            t90_idx = find(data_signal >= threshold_90, 1);
        else
            t10_idx = find(data_signal <= threshold_10, 1);
            t90_idx = find(data_signal <= threshold_90, 1);
        end
        
        if ~isempty(t10_idx) && ~isempty(t90_idx)
            rise_times(method_idx) = time_signal(t90_idx) - time_signal(t10_idx);
        else
            rise_times(method_idx) = NaN; 
        end
        
        error = abs(data_signal - Setpoint);
        tolerance = 0.02 * abs(Setpoint - Initial_point);
        last_outside_idx = find(error > tolerance, 1, 'last');
        
        if isempty(last_outside_idx)
            settling_times(method_idx) = 0; 
        elseif last_outside_idx == length(time_signal)
            settling_times(method_idx) = NaN; 
        else
            settling_times(method_idx) = time_signal(last_outside_idx + 1);
        end
        
        ss_errors(method_idx) = abs(Setpoint - data_signal(end)); 
        
        % Plotting directly in the loop
        plot(time_signal, data_signal, 'LineWidth', 1.5);
        
        % Move to the next simulation result in our batch
        sim_idx = sim_idx + 1; 
    end

    % --- Table and Plot Formatting ---
    Method_Name = method_names'; 
    ResultsTable = table(Method_Name, peak_times, overshoots, rise_times, settling_times, ss_errors, ...
        'VariableNames', {'Method_Name', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct', 'SteadyStateError'});
    disp(ResultsTable);

    % Plot Setpoint and the 2% Tolerance Bands for visual clarity
    yline(Setpoint, 'k--', 'Setpoint', 'LineWidth', 1.5);
    tol_val = 0.02 * abs(Setpoint - Initial_point);
    yline(Setpoint + tol_val, 'r:', 'LineWidth', 1);
    yline(Setpoint - tol_val, 'r:', 'LineWidth', 1);

    xlabel('Time (s)');
    ylabel('Position (Degrees)');
    title(fig_name);
    legend(method_names, 'Location', 'southeast');
    grid on;
end