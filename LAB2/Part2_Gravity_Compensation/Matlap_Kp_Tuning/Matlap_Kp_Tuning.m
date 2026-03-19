%% Basic Setup
clear;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.056427; 
% Trajectory [Initial_point, Setpoint]
Trajectory_Path = [0,90; 0,180; 0,270; 90,0; 180,0; 270,0];
rad2deg = 180/pi;
simfile = "C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Part2_Gravity_Compensation\Matlap_Kp_Tuning\Kp_Tuning.slx";

% Pre-allocate a table or structure to store multiple results
AllResults = table();
figure('Name', 'Trajectory Comparisons'); hold on;

%% --- Loop Through Trajectory Rows ---
for i = 1:size(Trajectory_Path, 1)
    % 1. Extract current Initial and Setpoint
    Initial_point = Trajectory_Path(i, 1);
    Setpoint = Trajectory_Path(i, 2);
    
    % 2. Push variables to base workspace for Simulink
    assignin('base', 'kp_value', kp_value);
    assignin('base', 'Initial_point', Initial_point);
    assignin('base', 'Setpoint', Setpoint);
    
    % 3. Run Simulation
    simout = sim(simfile);
    
    % 4. Extract data
    data_signal = simout.Position_Data.Data * rad2deg;
    time_signal = simout.Position_Data.Time;
    
    % 5. Calculate Metrics (Adaptive)
    if Setpoint > Initial_point
        [peak_val, peak_idx] = max(data_signal);
        overshoot = ((peak_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
    elseif Setpoint < Initial_point
        [peak_val, peak_idx] = min(data_signal);
        overshoot = ((Setpoint - peak_val) / abs(Setpoint - Initial_point)) * 100;
    else
        overshoot = 0; % No movement
        peak_idx = 1;
    end
    
    peak_time = time_signal(peak_idx);
    overshoot = max(0, overshoot); % Ensure non-negative
    
    % 6. Store results in Table
    CurrentRow = table(Initial_point, Setpoint, peak_time, overshoot, ...
        'VariableNames', {'Initial', 'Setpoint', 'PeakTime_s', 'PercentOvershoot'});
    AllResults = [AllResults; CurrentRow];
    
    % 7. Plot this specific run
    plot(time_signal, data_signal, 'DisplayName', sprintf('%d to %d', Initial_point, Setpoint));
end

%% --- Final Formatting ---
yline(0, 'k--'); % Baseline if needed
xlabel('Time (s)');
ylabel('Position (Degrees)');
title(sprintf('Trajectory Responses (k_p = %.4f)', kp_value));
legend('Location', 'best');
grid on;

disp('--- Final Results ---');
disp(AllResults);