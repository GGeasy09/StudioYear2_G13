
clear;
% R and L from experiment
motor_R = 3.341;
motor_L = 0.02587;

% Optimization's parameters
motor_B = 8.54E-06;
motor_Eff = 0.92042;
motor_Ke = 0.05438258655;
motor_J = 1/6382.937317;

%Mode Selection
load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_stair_v12_2000hz_time0.1_1.mat');
speed_Estimate_1 = double(speed);
time_1 = out.tout;
load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_stair_v12_2000hz_time0.1_2.mat');
speed_Estimate_2 = double(speed);
time_2 = out.tout;
load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_stair_v12_2000hz_time0.1_3.mat');
speed_Estimate_3 = double(speed);
time_3 = out.tout;


