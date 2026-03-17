%% Basic Setup
clear;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section (Single Run)
kp_value = 0.05; % Define the single value you want to test
Initial_point = 90; % ใส่ค่าจุดเริ่ม
Setpoint = 0; % ใส่ค่าจุดสุดท้าย
rad2deg = 180/pi;
simfile = "C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Part2_Gravity_Compensation\Matlap_Kp_Tuning\Kp_Tuning.slx";

% Ensure Simulink sees the kp_value by pushing it to the base workspace
assignin('base', 'kp_value', kp_value);

%% --- Simulation Run ---
simout = sim(simfile);
    
% Extract data
data_signal = simout.Position_Data.Data * rad2deg;
time_signal = simout.Position_Data.Time;

%% --- Calculate Metrics (Adaptive for Up/Down Step) ---
if Setpoint > Initial_point
    % Rising Step: Peak is the maximum value
    [peak_val, peak_idx] = max(data_signal);
    % Percent Overshoot formula for rising
    overshoot = ((peak_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
else
    % Falling Step: Peak is the minimum value
    [peak_val, peak_idx] = min(data_signal);
    % Percent Overshoot formula for falling (Setpoint - MinValue)
    overshoot = ((Setpoint - peak_val) / abs(Setpoint - Initial_point)) * 100;
end

% Common Metrics
peak_time = time_signal(peak_idx);

% Ensure overshoot isn't negative (in case it never crosses setpoint)
if overshoot < 0
    overshoot = 0; 
end

legend_label = sprintf('k_p = %.4f', kp_value);

%% --- Display Results Table ---
ResultsTable = table(kp_value, peak_time, overshoot, ...
    'VariableNames', {'Kp_Value', 'PeakTime_s', 'PercentOvershoot'});
disp(ResultsTable);

%% --- Figure: Position Plot ---
figure('Name', 'Single-Run Position');
plot(time_signal, data_signal, 'LineWidth', 1.5);
hold on;
yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Position (Degrees)');
title(sprintf('System Response for k_p = %.4f', kp_value));
legend(legend_label, 'Setpoint');
grid on;