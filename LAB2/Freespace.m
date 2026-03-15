clear;
run("Lab2_params_student.m");
% Define your known constants
R_Km = R/kt; % Replace with your actual calculated R/Km value

% Case pi: Kp*pi + R_Km <= 12
% Kp <= (12 - R_Km) / pi
bound1 = (12 - R_Km) / pi;

% Case -pi: Kp*(-pi) + R_Km >= -12
% -pi*Kp >= -12 - R_Km
% Kp <= (-12 - R_Km) / -pi  --> Sign flips!
bound2 = (-12 - R_Km) / -pi;

% Determine the intersection of the ranges
% Since both are "less than", the range is limited by the smaller value
upper_limit = min(bound1, bound2);

%kp from root herwit stability
root_bound = B*C/(A*kt);


fprintf('Range for Kp based on Case pi: Kp <= %.4f\n', bound1);
fprintf('Range for Kp based on Case -pi: Kp <= %.4f\n', bound2);
fprintf('--------------------------------------------\n');
fprintf('To satisfy BOTH cases, Kp must be <= %.4f\n', upper_limit);
disp(R_Km);
disp(root_bound);