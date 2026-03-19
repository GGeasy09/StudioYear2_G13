%% Basic Setup
clear;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section (Vectorized)
kp_vector = [0.04, 0.06, 0.08]; % Define the values you want to test
Initial_point = 0; %ใส่ค่าจุดเริ่ม
Setpoint = 360; %ใส่ค่าจุดสุดท้าย
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

%% --- Simulation Loop with Metrics ---
% ... (keep your pre-allocation code) ...

%% --- Updated Simulation Loop with Settling Time ---
results_ts = zeros(length(kp_vector), 1); % Pre-allocate

for i = 1:length(kp_vector)
    kp_value = kp_vector(i); 
    simout = sim(simfile);
    
    data_signal = simout.Position_Data.Data * rad2deg;
    time_signal = simout.Position_Data.Time;
    
    results_pos{i} = data_signal;
    results_time{i} = time_signal;
    
    % --- Peak Time and Overshoot ---
    [max_val, max_idx] = max(data_signal);
    peak_times(i) = time_signal(max_idx);
    overshoots(i) = ((max_val - Setpoint) / abs(Setpoint - Initial_point)) * 100;
    if overshoots(i) < 0, overshoots(i) = 0; end
    
    % --- Settling Time (2% Threshold) ---
    threshold = 0.02 * abs(Setpoint - Initial_point);
    idx_outside = find(abs(data_signal - Setpoint) > threshold);
    if isempty(idx_outside)
        results_ts(i) = 0;
    elseif idx_outside(end) == length(data_signal)
        results_ts(i) = NaN; % Didn't settle within simulation time
    else
        results_ts(i) = time_signal(idx_outside(end));
    end
    
    legend_labels{i} = sprintf('k_p = %.4f', kp_value);
end

%% --- Figure Update: Adding Ts to Labels ---
figure('Name', 'Multi-Run Position Comparison'); hold on;
colors = lines(length(kp_vector));
for i = 1:length(kp_vector)
    plot(results_time{i}, results_pos{i}, 'LineWidth', 1.5, 'Color', colors(i,:));
    
    [peak_val, idx] = max(results_pos{i});
    peak_x = results_time{i}(idx);
    
    % Updated label string with Ts
    label_str = sprintf('  Kp: %.3f\n  OS: %.1f%%\n  Tp: %.2fs\n  Ts: %.2fs', ...
                kp_vector(i), overshoots(i), peak_times(i), results_ts(i));
    
    text(peak_x, peak_val, label_str, 'Color', colors(i,:), ...
         'VerticalAlignment', 'bottom', 'FontSize', 8, 'FontWeight', 'bold');
end
% --- Your Original Title Restored ---
title_str = sprintf('System Response for Multiple k_p Values (From %d to %d Degrees)', ...
            Initial_point, Setpoint);
title(title_str);yline(Setpoint, 'k:', 'Setpoint');
grid on; xlabel('Time (s)'); ylabel('Position (Deg)');