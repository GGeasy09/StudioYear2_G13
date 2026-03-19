%% Basic Setup
clear; clc;
% Load your base parameters
run("C:\Users\USER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 3.6237;
ki_value = 1.3254;
kd_value = 1.8856;
rad2deg = 180/pi;

% For parsim, just use the model name (make sure your current folder is set correctly in MATLAB)
model_name = 'Validation'; 
load_system("C:\Users\USER\Documents\GitHub\StudioYear2_G13\LAB2\Part3_PID_Design\Tuning&Validation\Validation.slx");

Control_Method = [1, 2, 3];
num_methods = length(Control_Method);

test_initials  = [0,   0,   0,  90, 180, 270];
test_setpoints = [90, 180, 270,   0,   0,   0];
num_tests = length(test_initials);

%% --- STEP 1: Build the Parallel Simulation Batch ---
fprintf('Building parallel simulation inputs...\n');
total_sims = num_tests * num_methods;

% We will initialize the array backwards to safely pre-allocate it in memory
for k = total_sims:-1:1
    simInputs(k) = Simulink.SimulationInput(model_name);
end

sim_idx = 1;
for test_idx = 1:num_tests
    for method_idx = 1:num_methods
        % Assign the specific variables for THIS specific simulation run
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Initial_point', test_initials(test_idx));
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Setpoint', test_setpoints(test_idx));
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('Controller_Method', Control_Method(method_idx));
        
        % Parallel workers don't share your base workspace, so we must pass the PID gains to them
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('kp_value', kp_value);
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('ki_value', ki_value);
        simInputs(sim_idx) = simInputs(sim_idx).setVariable('kd_value', kd_value);
        
        sim_idx = sim_idx + 1;
    end
end

%% --- STEP 2: Run All Simulations Simultaneously ---
fprintf('Running %d simulations in parallel. This may take a moment to start the parallel pool...\n', total_sims);
% parsim will automatically use all available CPU cores!
simOutputs = parsim(simInputs, 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on');

%% --- STEP 3: Process Results, Print Tables, and Plot ---
fprintf('\nProcessing results...\n');

% We use a standard loop here because the math and plotting are instant. 
% We are just extracting data from the simOutputs array we already generated.
sim_idx = 1; 

for test_idx = 1:num_tests
    Initial_point = test_initials(test_idx);
    Setpoint = test_setpoints(test_idx);
    
    fprintf('\n======================================================\n');
    fprintf('TEST CASE %d: Stepping from %d° to %d°\n', test_idx, Initial_point, Setpoint);
    fprintf('======================================================\n');
    
    % Arrays for this specific test case table
    peak_times = zeros(num_methods, 1);
    overshoots = zeros(num_methods, 1);
    rise_times = zeros(num_methods, 1);     
    settling_times = zeros(num_methods, 1); 
    ss_errors = zeros(num_methods, 1);      
    legend_labels = cell(num_methods, 1);
    
    fig_name = sprintf('Control Comparison: %d° to %d°', Initial_point, Setpoint);
    figure('Name', fig_name, 'Color', 'w');
    hold on;

    for i = 1:num_methods
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
        
        peak_times(i) = time_signal(peak_idx);
        overshoots(i) = max(0, os_calc);
        
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
            rise_times(i) = time_signal(t90_idx) - time_signal(t10_idx);
        else
            rise_times(i) = NaN; 
        end
        
        error = abs(data_signal - Setpoint);
        tolerance = 0.02 * abs(Setpoint - Initial_point);
        last_outside_idx = find(error > tolerance, 1, 'last');
        
        if isempty(last_outside_idx)
            settling_times(i) = 0; 
        elseif last_outside_idx == length(time_signal)
            settling_times(i) = NaN; 
        else
            settling_times(i) = time_signal(last_outside_idx + 1);
        end
        
        ss_errors(i) = abs(Setpoint - data_signal(end)); 
        legend_labels{i} = sprintf('Method %d', Control_Method(i));
        
        % Plotting directly in the loop
        plot(time_signal, data_signal, 'LineWidth', 1.5);
        
        % Move to the next simulation result in our batch
        sim_idx = sim_idx + 1; 
    end

    % --- Table and Plot Formatting ---
    ResultsTable = table(Control_Method', peak_times, overshoots, rise_times, settling_times, ss_errors, ...
        'VariableNames', {'Method_ID', 'PeakTime_s', 'PercentOvershoot', 'RiseTime_s', 'SettlingTime_2pct', 'SteadyStateError'});
    disp(ResultsTable);

    yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel('Position (Degrees)');
    title(fig_name);
    legend(legend_labels, 'Location', 'southeast');
    grid on;
end