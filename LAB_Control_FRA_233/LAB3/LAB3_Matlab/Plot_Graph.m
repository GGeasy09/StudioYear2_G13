clear; close all;

% Load the data
load("LAB3_Matlab_Result\Part2\exp1.mat");

% Extract elements
data1 = data.getElement("Offset_Pos1");
data2 = data.getElement("Offset_Pos2");
data3 = data.getElement("Offset_Pos3");
data4 = data.getElement("Offset_Distance");
% pos1 = data.getElement("Velocity1");
% pos2 = data.getElement("Velocity2");
% pos3 = data.getElement("Velocity3");

% Extract Time (assuming all datasets share the same time vector)
Time = data1.Values.Time;


% Extract and convert values from cm to meters
% Using 0.01 conversion factor
pos1 = data1.Values.Data * 0.01;
pos2 = data2.Values.Data * 0.01;
pos3 = data3.Values.Data * 0.01;
dist = data4.Values.Data * 0.01;

% Plotting
figure;
hold on; % Keep all plots on the same axes

plot(Time, pos1, 'LineWidth', 1.5);
plot(Time, pos2, 'LineWidth', 1.5);
plot(Time, pos3, 'LineWidth', 1.5);
plot(Time, dist, '--', 'LineWidth', 1.5); % Using dashed line for total distance

% Formatting the Graph
grid on;
xlabel('Time (s)');
ylabel('Kalman Gain');
title('Kalman Gain');

legend('Q(7.5e-9)', ...
       'Q(7.5e-6)', ...
       'Q(7.5e-3)', ...
       'Raw Data', ...
       'Location', 'best');

hold off;