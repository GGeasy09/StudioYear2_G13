% clear;

% R and L from experiment
motor_R = 3.341;
motor_L = 0.002587;

% Optimization's parameters
motor_B = 0.00018169;
motor_Eff = 0.92335;
motor_Ke = 0.033197;
motor_J = 8064.2;

% model = 'Lab1_parameter_estimation_student';
% simOut = sim(model);
% Predict_Value = simOut.Estimate_Result.signals.values;
% time = simOut.Estimate_Result.time;
% 
% load('Lab1 Part2 Data\Lab1 Part2 Data\lap1_ramp_v12_100hz_slope0.1_1.mat');
Raw_Value = speed;

% Ensure vectors are column and same length
Predict_Value = Predict_Value(:);
Raw_Value = Raw_Value(:);

n = min(numel(Predict_Value), numel(Raw_Value));
Predict_Value = Predict_Value(1:n);
Raw_Value = Raw_Value(1:n);

% Compute mean of sum squared errors
sse = mean((Raw_Value - Predict_Value).^2);

disp(sse);