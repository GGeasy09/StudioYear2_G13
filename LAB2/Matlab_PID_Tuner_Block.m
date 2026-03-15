clear;
run("Lab2_params_student.m");

Initial_point = 90;
Setpoint = 90; % 0 - 360 Degrees
feed_forward = 1; %with feeddorward 1 no feed forward 0

simout = sim("Simulink_PID_Tuner_Block.slx");

posdata = simout.Position_Data;
velodata = simout.Velocity_Data;
control_effort = simout.control_effort;
% Plot the position data against time
time = simout.tout; % Assuming Time is part of the simout structure
figure;
plot(posdata);
xlabel('Time (s)');
ylabel('Position (Degrees)');
title('Position Data over Time');
grid on;

% 1. Extract raw numeric data
data = posdata.Data;
t = posdata.Time;
steadyState = data(end); 

% 2. Determine Direction and Find Peak
if Setpoint < Initial_point
    % Scenario: 90 -> 0 (Step Down)
    % The overshoot is the lowest point (Minimum)
    [peakValue, idx] = min(data);
    peakType = 'Undershoot Peak';
else
    % Scenario: 0 -> 90 (Step Up)
    % The overshoot is the highest point (Maximum)
    [peakValue, idx] = max(data);
    peakType = 'Overshoot Peak';
end

peakTime = t(idx);

% 3. Calculate Percentage Overshoot (%OS)
% We use the total change (Step Size) as the denominator to avoid 
% division by zero if the Setpoint is 0.
stepSize = abs(Setpoint - Initial_point);
overshoot = (abs(peakValue - steadyState) / stepSize) * 100;

% Logic check: If the "peak" is just the starting point, OS is 0%
if idx == 1
    overshoot = 0;
end

% --- Display Updated Results ---
fprintf('Direction: %s\n', peakType);
fprintf('Peak Time (Tp): %.4f seconds\n', peakTime);
fprintf('Peak Value: %.4f Degrees\n', peakValue);
fprintf('Percentage Overshoot (%%OS): %.2f%%\n', overshoot);
% Optional: Plot markers on the graph to verify
hold on;
plot(peakTime, peakValue, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(peakTime, peakValue, '  Peak', 'Color', 'r');
yline(steadyState, '--r', 'Steady State');

%% --- Figure 2: Velocity Data ---
figure;
plot(velodata.Time, velodata.Data, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (Degrees/s)');
title('System Velocity over Time');
grid on;

%% --- Figure 3: Control Effort ---
figure;
% Using a different color (Red or Green) to distinguish from position
plot(control_effort.Time, control_effort.Data, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Effort (V or PWM)');
title('Controller Output (Control Effort)');
grid on;

% Add a horizontal line at 0 for control effort to see push vs. pull
hold on;
yline(0, '--k', 'Zero Effort');