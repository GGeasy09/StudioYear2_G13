clear;
% R and L from experiment
motor_R = 3.341;
motor_L = 0.002587;

%% Ramp Selection
% Optimization's parameters 
motor_B = 3.90E-06;
motor_Eff = 0.8279549717;
motor_Ke = 6382.937317;
motor_J = 0.05438258655;
load('Lab1 Part2 Parameter\Step_12V.mat');
load('Lab1 Part2 Data\Lab1 Part2 Data Solve Error\lap1_step_v12_2000hz_1.mat')
% Execute the simulation and store results

% 
% motor_B = 8.86E-05;
% motor_Eff = 0.60715;
% motor_Ke = 17144;
% motor_J = 0.041039;
% load('Lab1 Part2 Parameter\Step_9V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v9_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_vต_2000hz_3.mat');
% 
% motor_B = 0.00058127;
% motor_Eff = 0.6063;
% motor_Ke = 146650;
% motor_J = 0.065192;
% load('Lab1 Part2 Parameter\Step_6V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v6_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v6_2000hz_3.mat');
% 
% %% Step Selection
% % Optimization's parameters 
% motor_B = 3.90E-06;
% motor_Eff = 0.8279549717;
% motor_Ke = 6382.937317;
% motor_J = 0.05438258655;
% load('Lab1 Part2 Parameter\Stair_wait0.5.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v12_2000hz_2.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v12_2000hz_2.mat');
% 
% motor_B = 8.86E-05;
% motor_Eff = 0.60715;
% motor_Ke = 17144;
% motor_J = 0.041039;
% load('Lab1 Part2 Parameter\Step_9V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v9_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_vต_2000hz_3.mat');
% 
% motor_B = 0.00058127;
% motor_Eff = 0.6063;
% motor_Ke = 146650;
% motor_J = 0.065192;
% load('Lab1 Part2 Parameter\Step_6V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v6_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v6_2000hz_3.mat');
% 
% %% Sine Selection
% % Optimization's parameters 
% motor_B = 3.90E-06;
% motor_Eff = 0.8279549717;
% motor_Ke = 6382.937317;
% motor_J = 0.05438258655;
% load('Lab1 Part2 Parameter\Stair_wait0.5.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v12_2000hz_2.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v12_2000hz_2.mat');
% 
% motor_B = 8.86E-05;
% motor_Eff = 0.60715;
% motor_Ke = 17144;
% motor_J = 0.041039;
% load('Lab1 Part2 Parameter\Step_9V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v9_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_vต_2000hz_3.mat');
% 
% motor_B = 0.00058127;
% motor_Eff = 0.6063;
% motor_Ke = 146650;
% motor_J = 0.065192;
% load('Lab1 Part2 Parameter\Step_6V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v6_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v6_2000hz_3.mat');
% 
% %% Chirp Selection
% % Optimization's parameters 
% motor_B = 3.90E-06;
% motor_Eff = 0.8279549717;
% motor_Ke = 6382.937317;
% motor_J = 0.05438258655;
% load('Lab1 Part2 Parameter\Stair_wait0.5.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v12_2000hz_2.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v12_2000hz_2.mat');
% 
% motor_B = 8.86E-05;
% motor_Eff = 0.60715;
% motor_Ke = 17144;
% motor_J = 0.041039;
% load('Lab1 Part2 Parameter\Step_9V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v9_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_vต_2000hz_3.mat');
% 
% motor_B = 0.00058127;
% motor_Eff = 0.6063;
% motor_Ke = 146650;
% motor_J = 0.065192;
% load('Lab1 Part2 Parameter\Step_6V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v6_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v6_2000hz_3.mat');
% 
% %% Stair Step Selection
% % Optimization's parameters 
% motor_B = 3.90E-06;
% motor_Eff = 0.8279549717;
% motor_Ke = 6382.937317;
% motor_J = 0.05438258655;
% load('Lab1 Part2 Parameter\Stair_wait0.5.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v12_2000hz_2.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v12_2000hz_2.mat');
% 
% motor_B = 8.86E-05;
% motor_Eff = 0.60715;
% motor_Ke = 17144;
% motor_J = 0.041039;
% load('Lab1 Part2 Parameter\Step_9V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v9_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_vต_2000hz_3.mat');
% 
% motor_B = 0.00058127;
% motor_Eff = 0.6063;
% motor_Ke = 146650;
% motor_J = 0.065192;
% load('Lab1 Part2 Parameter\Step_6V.mat');
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_step_v6_2000hz_3.mat');
% save('Lab1 Part2 before_Analyse\lap1_step_v6_2000hz_3.mat');