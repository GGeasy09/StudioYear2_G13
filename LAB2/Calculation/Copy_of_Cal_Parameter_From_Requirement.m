% Calculate Natural Frequency (Wn) from Settling Time and %OS
clear; clc;
run("Lab2_params_student.m");

%% 1. Input known parameters
OS = 5;           % Percentage Overshoot
ts = 3;           % Settling Time (seconds) - 2% criterion

%% 2. Calculate damping ratio (zeta)
L = log(OS/100);
zeta = -L / sqrt(pi^2 + L^2);

%% 3. Calculate natural frequency (Wn)
% Derived from: ts = 4 / (zeta * Wn)
Wn = 4 / (ts * zeta);

%% 4. Results
fprintf('\n--- Results ---\n');
fprintf('Damping Ratio (zeta): %.4f\n', zeta);
fprintf('Natural Frequency (Wn): %.4f rad/s\n', Wn);

%% 5. Calculate kp from the Pole
% The given pole (typically s = -zeta*Wn + j*Wn*sqrt(1-zeta^2))
% Based on your previous pole:
s = -0.7674 + 1.2984j; 

% The characteristic equation: A*s^3 + B*s^2 + C*s + kp*kt = 0
% Solving for kp:
kp = -(A*s^3 + B*s^2 + C*s) / kt;

fprintf('The calculated kp is: %.4f\n', real(kp));

% Verification
if abs(imag(kp)) > 1e-3
    warning('The result has a significant imaginary part (%.4fi).', imag(kp));
end