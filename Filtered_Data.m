clear;
%% Extract Variable 
load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v12_2000hz_1.mat');
load('Lab1 Part2 Parameter\Step_12V.mat');
% Assuming 'speed' is a variable loaded from the .mat file, we can proceed to analyze or process it.



dval = diff(speed);              % rate of change
idx = find(dval > 0, 1, 'first');  % first point it rises

start_time = time(idx)

figure;
plot(time, speed)
hold on
xline(start_time, 'r')
hold off
