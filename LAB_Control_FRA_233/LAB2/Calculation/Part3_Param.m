% Calculate Natural Frequency (Wn) from Settling Time and %OS
clear; clc;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% 1. Input known parameters
OS = 5;           % Percentage Overshoot (%)
ts = 2.5;         % Settling Time (seconds) - 2% criterion

%% 2. Calculate Damping Ratio (zeta)
L = log(OS/100);
zeta = -L / sqrt(pi^2 + L^2);

%% 3. Calculate Natural Frequency (Wn)
% Derived from: ts = 4 / (zeta * Wn)
Wn = 4 / (ts * zeta);

%% 4. Calculate Additional Parameters
% Damped Natural Frequency
Wd = Wn * sqrt(1 - zeta^2);

% Peak Time (Tp)
Tp = pi / Wd;

% Rise Time (Tr) - using standard approximation
Tr = (1 + 1.1*zeta + 1.4*zeta^2) / Wn;

% Dominant Pole Locations
real_part = -zeta * Wn;
imag_part = Wd;

%% 5. Display Results
fprintf('\n--- Calculated System Parameters ---\n');
fprintf('Damping Ratio (zeta):         %.4f\n', zeta);
fprintf('Natural Frequency (Wn):       %.4f rad/s\n', Wn);
fprintf('Damped Natural Frequency (Wd): %.4f rad/s\n', Wd);
fprintf('Peak Time (Tp):               %.4f s\n', Tp);
fprintf('Rise Time (Tr):               %.4f s\n', Tr);
fprintf('Dominant Poles:               %.4f ± %.4fj\n', real_part, imag_part);

%%Cal PI
% 1. Define your point and plant constants (ensure A, B, C, kt are defined)
s_point = -1.6 + 1.68j; 

% 2. Calculate the "Magnitude" of Kp 
% We take the absolute value because Gain must be a real number.
% If s_point is on the root locus, the imaginary part will be near zero anyway.
zero_pos = 1.6-1.68/tand(37.86);
disp(zero_pos);
num_val = s_point * (A*s_point^2 + B*s_point + C);
den_val = kt * (s_point - 0.561);

% Based on your equation: Kp = - (Plant_Denominator) / (Controller_Numerator)
% Note: Usually we use the absolute value to find the required gain
kp_complex = - (s_point * (A*s_point^2 + B*s_point + C)) / (kt * (s_point - 0.561));
kp = abs(kp_complex); 

% 3. Calculate Ki using your ratio
ki = - 0.561 * kp;

%% Display Results
fprintf('Calculated Kp: %.4f\n', kp);
fprintf('Calculated Ki: %.4f\n', ki);