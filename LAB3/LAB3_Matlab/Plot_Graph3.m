clear; close all;

% Load data
load("LAB3_Matlab_Result\Part3\Extract2.mat");
% offset = 0.87;
offset = 1.1;

% Extract raw numeric arrays from the timeseries objects and convert cm to m (/100)
pos1_raw = data.getElement("Origin_Pos1").Values.Data / 100;
pos2_raw = data.getElement("Origin_Pos2").Values.Data / 100;
pos3_raw = data.getElement("Origin_Pos3").Values.Data / 100;
data_distance_raw = data.getElement("Offset_Distance").Values.Data / 100;

% Extract velocity and convert cm/s to m/s (/100)
velo1_raw = data.getElement("Velocity1").Values.Data / 100;
velo2_raw = data.getElement("Velocity2").Values.Data / 100;
velo3_raw = data.getElement("Velocity3").Values.Data / 100;

% Extract Model data exactly as it is (no offset, no conversion)
Model_pos = data.getElement("x").Values.Data;
Model_velo = data.getElement("x_dot").Values.Data;
Model_Time = data.getElement("x").Values.Time;

% Extract time array from one of the timeseries objects
Time_raw = data.getElement("Offset_Distance").Values.Time;

%% --- Pull back data to offset ---
% Find the indices where the raw time is greater than or equal to the offset
idx = Time_raw >= offset;

% Shift the time array so that t=offset becomes t=0
Time = Time_raw(idx) - offset;

% Filter all position, distance, and velocity arrays using the same indices
pos1 = pos1_raw(idx);
pos2 = pos2_raw(idx);
pos3 = pos3_raw(idx);
data_distance = data_distance_raw(idx);

velo1 = velo1_raw(idx);
velo2 = velo2_raw(idx);
velo3 = velo3_raw(idx);

%% --- Plotting ---

%% Plot 1: Model Velocity vs Velocity 1 (Q = 7.5)
figure('Name', 'Model Velo vs Velo1');
hold on;
plot(Time, velo1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Velocity 1 (Q = 7.5)');
plot(Model_Time, Model_velo, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Velocity');
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Model Velocity vs. Kalman Filter Velocity (Q = 7.5)');
legend('Location', 'best');
grid on;

%% Plot 2: Model Velocity vs Velocity 2 (Q = 75)
figure('Name', 'Model Velo vs Velo2');
hold on;
plot(Time, velo2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Velocity 2 (Q = 75)');
plot(Model_Time, Model_velo, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Velocity');
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Model Velocity vs. Kalman Filter Velocity (Q = 75)');
legend('Location', 'best');
grid on;

%% Plot 3: Model Velocity vs Velocity 3 (Q = 750)
figure('Name', 'Model Velo vs Velo3');
hold on;
plot(Time, velo3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Velocity 3 (Q = 750)');
plot(Model_Time, Model_velo, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Velocity');
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Model Velocity vs. Kalman Filter Velocity (Q = 750)');
legend('Location', 'best');
grid on;

%% Plot 1: Model Position vs Position 1 (Q = 7.5)
figure('Name', 'Model Pos vs Pos1');
hold on;
plot(Time, pos1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Position 1 (Q = 7.5)');
plot(Model_Time, Model_pos, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Position');
hold off;
xlabel('Time (s)');
ylabel('Position (m)');
title('Model Position vs. Kalman Filter Position (Q = 7.5)');
legend('Location', 'best');
grid on;

%% Plot 2: Model Position vs Position 2 (Q = 75)
figure('Name', 'Model Pos vs Pos2');
hold on;
plot(Time, pos2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Position 2 (Q = 75)');
plot(Model_Time, Model_pos, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Position');
hold off;
xlabel('Time (s)');
ylabel('Position (m)');
title('Model Position vs. Kalman Filter Position (Q = 75)');
legend('Location', 'best');
grid on;

%% Plot 3: Model Position vs Position 3 (Q = 750)
figure('Name', 'Model Pos vs Pos3');
hold on;
plot(Time, pos3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Position 3 (Q = 750)');
plot(Model_Time, Model_pos, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Model Position');
hold off;
xlabel('Time (s)');
ylabel('Position (m)');
title('Model Position vs. Kalman Filter Position (Q = 750)');
legend('Location', 'best');
grid on;