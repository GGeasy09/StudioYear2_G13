clear; close all;

% Load data
load("LAB3_Matlab_Result\Part3\exp2_10_4cm.mat");

% Extract raw numeric arrays from the timeseries objects and convert cm to m (/100)
pos1 = data.getElement("Origin_Pos1").Values.Data / 100;
pos2 = data.getElement("Origin_Pos2").Values.Data / 100;
pos3 = data.getElement("Origin_Pos3").Values.Data / 100;
data_distance = data.getElement("Offset_Distance").Values.Data / 100;

% Extract velocity and convert cm/s to m/s (/100)
velo1 = data.getElement("Velocity1").Values.Data / 100;
velo2 = data.getElement("Velocity2").Values.Data / 100;
velo3 = data.getElement("Velocity3").Values.Data / 100;

% Extract time array from one of the timeseries objects
Time = data.getElement("Offset_Distance").Values.Time;

%% Plot 1: Position 1 and Distance
figure('Name', 'KALMAN Filter Q = 7.5');
hold on;
plot(Time, pos1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Q = 7.5');
plot(Time, data_distance, 'k--', 'LineWidth', 1.5, 'DisplayName', 'raw data');
hold off;
xlabel('Time (s)');
ylabel('Position / Distance (m)');
title('KALMAN Filter Q = 7.5 vs Raw Data');
legend('Location', 'best');
grid on;

%% Plot 2: Position 2 and Distance
figure('Name', 'KALMAN Filter Q = 75');
hold on;
plot(Time, pos2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Q = 75');
plot(Time, data_distance, 'k--', 'LineWidth', 1.5, 'DisplayName', 'raw data');
hold off;
xlabel('Time (s)');
ylabel('Position / Distance (m)');
title('KALMAN Filter Q = 75 vs Raw Data');
legend('Location', 'best');
grid on;

%% Plot 3: Position 3 and Distance
figure('Name', 'KALMAN Filter Q = 750');
hold on;
plot(Time, pos3, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Q = 750');
plot(Time, data_distance, 'k--', 'LineWidth', 1.5, 'DisplayName', 'raw data');
hold off;
xlabel('Time (s)');
ylabel('Position / Distance (m)');
title('KALMAN Filter Q = 750 vs Raw Data');
legend('Location', 'best');
grid on;

%% Plot 4: Position (Q=7.5) and Velocity
figure('Name', 'Velo1');
yyaxis left;
plot(Time, pos1, 'b-', 'LineWidth', 1.5);
ylabel('Position (Q = 7.5) (m)');
yyaxis right;
plot(Time, velo1, 'r-', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
xlabel('Time (s)');
title('Position (Q = 7.5) & Velocity vs. Time');
grid on;

%% Plot 5: Position (Q=75) and Velocity
figure('Name', 'Velo2');
yyaxis left;
plot(Time, pos2, 'g-', 'LineWidth', 1.5);
ylabel('Position (Q = 75) (m)');
yyaxis right;
plot(Time, velo2, 'r-', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
xlabel('Time (s)');
title('Position (Q = 75) & Velocity vs. Time');
grid on;

%% Plot 6: Position (Q=750) and Velocity
figure('Name', 'Velo3');
yyaxis left;
plot(Time, pos3, 'm-', 'LineWidth', 1.5);
ylabel('Position (Q = 750) (m)');
yyaxis right;
plot(Time, velo3, 'r-', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
xlabel('Time (s)');
title('Position (Q = 750) & Velocity vs. Time');
grid on;