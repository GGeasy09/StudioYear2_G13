clear;
run("Lab2_params_student.m");
format long

% --- Input Parameters ---
zeta = 0.6901;      % Damping Ratio
wn = 2.31855;        % Natural Frequency (rad/s)

% --- 1. S-Plane Calculations ---
sigma = zeta * wn;                   % Real part (decay constant)
wd = wn * sqrt(1 - zeta^2);          % Damped frequency
theta_deg = acosd(zeta);             % Angle in degrees

% --- 2. Time Domain Performance ---
OS = exp(-(zeta * pi) / sqrt(1 - zeta^2)) * 100; % % Overshoot
Ts_2pct = 4 / sigma;                             % 2% Settling time
Tp = pi / wd;                                    % Peak time
% Approximation for Rise Time (0 to 100%)
Tr = (pi - acos(zeta)) / wd;

% --- Display Results ---
fprintf('--- S-Plane Coordinates ---\n')
fprintf('Real Part (sigma): %.2f\n', sigma)
fprintf('Imaginary Part (wd): %.2f\n', wd)
fprintf('Angle (theta): %.2f degrees\n', theta_deg)
fprintf('--- Time Response Metrics ---\n')
fprintf('Percent Overshoot: %.2f%%\n', OS)
fprintf('Settling Time (2%%): %.2f sec\n', Ts_2pct)
fprintf('Peak Time: %.2f sec\n', Tp)
fprintf('Rise Time: %.2f sec\n', Tr)

s_point = -1.6 + 1.68j; % Replace with your actual point

angle_deg = angle(G_val) * 180/pi
controlSystemDesigner("rlocus",sys_tf);