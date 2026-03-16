clear;
run("Lab2_params_student.m");
%initial - setpoint
% 0 - 90
% 90 - 90
% 90 - 0
Initial_point = 0; % Target in degrees
Setpoint = 90;      
kp_value = 0.0788;
rad2deg = 180/pi; % Conversion constant

%% --- Run 1: Feed-Forward ON ---
feed_forward = 1;
simout1 = sim("Simulink_Hand_PID_Tuning.slx");
% Convert data to degrees immediately
pos1_deg = simout1.Position_Data.Data * rad2deg;
vel1_deg = simout1.Velocity_Data.Data * rad2deg;
time1 = simout1.Position_Data.Time;

%% --- Run 2: Feed-Forward OFF ---
feed_forward = 0;
simout2 = sim("Simulink_Hand_PID_Tuning.slx");
% Convert data to degrees immediately
pos2_deg = simout2.Position_Data.Data * rad2deg;
vel2_deg = simout2.Velocity_Data.Data * rad2deg;
time2 = simout2.Position_Data.Time;

%% --- Figure 1: Position Comparison (Degrees) ---
figure('Name', 'Position Comparison');
plot(time1, pos1_deg, 'b', 'LineWidth', 1.5); hold on;
plot(time2, pos2_deg, 'r--', 'LineWidth', 1.5);

% Use the Setpoint variable to draw the horizontal line
yline(Setpoint, 'k:', sprintf('Setpoint (%g°)', Setpoint), 'LineWidth', 1.2);

xlabel('Time (s)');
ylabel('Position (Degrees)');
title('Position: Feed-Forward Comparison');
legend('FF ON', 'FF OFF', 'Target');
grid on;

%% --- Figure 2: Velocity Comparison (Degrees/s) ---
figure('Name', 'Velocity Comparison');
plot(time1, vel1_deg, 'b', 'LineWidth', 1.5); hold on;
plot(time2, vel2_deg, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (Degrees/s)');
title('Velocity: Feed-Forward Comparison');
legend('FF ON', 'FF OFF');
grid on;

%% --- Figure 3: Control Effort ---
figure('Name', 'Control Effort');
plot(simout1.control_effort.Time, simout1.control_effort.Data, 'b', 'LineWidth', 1.5); hold on;
plot(simout2.control_effort.Time, simout2.control_effort.Data, 'r--', 'LineWidth', 1.5);
ylabel('Control Effort (V)');
xlabel('Time (s)');
title('Controller Output');
legend('FF ON', 'FF OFF');
grid on;

%%Find Overshoot
% 1. Extract raw numeric data
data = simout1.Position_Data.Data * rad2deg;
t = simout1.Position_Data.Time;
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

% Feedforward_Effort = simout1.