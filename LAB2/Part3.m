% Calculate Natural Frequency (Wn) from Peak Time and Zeta
clear; clc;
run("Lab2_params_student.m");
% 1. Input known parameters
OS = 10;
tp = 3;

% 2. Calculate damping ratio (zeta)
L = log(OS/100);
zeta = -L / sqrt(pi^2 + L^2);

% 3. Calculate natural frequency (Wn)
% Derived from: Wn = pi / (tp * sqrt(1 - zeta^2))
Wn = pi / (tp * sqrt(1 - zeta^2));

% 4. Results
fprintf('\n--- Results ---\n');
fprintf('Damping Ratio (zeta): %.4f\n', zeta);
fprintf('Natural Frequency (Wn): %.4f rad/s\n', Wn);

% The given pole
s = -0.7674 + 1.2984j;

% The characteristic equation is: A*s^3 + B*s^2 + C*s + kp*km = 0
% Solving for kp:
kp = -(A*s^3 + B*s^2 + C*s) / kt;

% Display result
% Note: kp should ideally be a real number. If there is a tiny 
% imaginary part (e.g., 0.00001i), it is likely due to rounding 
% in the pole value provided.
fprintf('The calculated kp is: %.4f\n', real(kp));

% Verification: Check if the magnitude of the imaginary part is negligible
if abs(imag(kp)) > 1e-3
    warning('The result has a significant imaginary part (%.4fi). Check your constants or pole value.', imag(kp));
end