clear;
run("Lab2_params_student.m");

% 1. Inputs (Replace these with your actual values)
OS_percent = 10;    % Percentage Overshoot
Tp = 3;           % Peak Time in seconds

% 2. Calculate Damping Ratio (zeta) first
OS_frac = OS_percent / 100;
zeta = -log(OS_frac) / sqrt(pi^2 + log(OS_frac)^2);

% 3. Calculate Natural Frequency (wn)
wn = pi / (Tp * sqrt(1 - zeta^2));

% 4. Display Result
fprintf('Damping Ratio (zeta): %.4f\n', zeta);
fprintf('Natural Frequency (wn): %.4f rad/s\n', wn);

%kp from OS
kp_OS = C*C/(4*B*kt*zeta*zeta);
fprintf('Proportional Gain (kp_OS) must under: %.4f\n', kp_OS);

%kp from Peaktime
kp_Pt = B*wn*wn/kt;
fprintf('Proportional Gain (kp_Peaktime) must over: %.4f\n', kp_Pt);