
clear;
% R and L from experiment
motor_R = 3.341;
motor_L = 0.02587;

% Optimization's parameters
motor_B = 8.54E-06;
motor_Eff = 0.92042;
motor_Ke = 0.05438258655;
motor_J = 1/6382.937317;

load('Lab1 Part2 Parameter\Step_12V.mat');

%Mode Selection
load('Lab1 Part2 Data\Lab1 Part2 Data Solve Error\lap1_step_v12_2000hz_1.mat');
speed_Estimate_1 = double(raw_data_shift);
time_1 = time_shift;
load('Lab1 Part2 Data\Lab1 Part2 Data Solve Error\lap1_step_v12_2000hz_2.mat');
speed_Estimate_2 = double(raw_data_shift);
time_2 = time_shift;
load('Lab1 Part2 Data\Lab1 Part2 Data Solve Error\lap1_step_v12_2000hz_3.mat');
speed_Estimate_3 = double(raw_data_shift);
time_3 = time_shift;


