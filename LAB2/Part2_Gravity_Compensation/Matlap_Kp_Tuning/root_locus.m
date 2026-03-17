clear;
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

% --- Input Parameters ---
zeta = 0.5912;      % Damping Ratio
wn = 1.2978;         % Natural Frequency (rad/s)

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
fprintf('Angle (theta): %.2f degrees\n\n', theta_deg)

fprintf('--- Time Response Metrics ---\n')
fprintf('Percent Overshoot: %.2f%%\n', OS)
fprintf('Settling Time (2%%): %.2f sec\n', Ts_2pct)
fprintf('Peak Time: %.2f sec\n', Tp)
fprintf('Rise Time: %.2f sec\n', Tr)

s_point = -0.9191 + 1.2532j; % Replace with your actual point
tf = s_point*s_point*(A*s_point^2+B*s_point+C)/(kt*(s+2.612));


angle_deg = angle(G_val) * 180/pi
controlSystemDesigner("rlocus",sys_tf);