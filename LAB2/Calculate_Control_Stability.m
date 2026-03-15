clear;
run("Lab2_params_student.m"); % Ensure this defines B, C, and kt
kp = 64.4873;

% 1. Calculate intermediate parameters
wn = sqrt((kp * kt) / B);
zeta = C / (2 * sqrt(B * kp * kt));

% 2. Check if the system is underdamped
if zeta < 1
    % Calculate Peak Time
    tp = pi / (wn * sqrt(1 - zeta^2));
    
    % Calculate Percentage Overshoot
    OS = exp((-zeta * pi) / sqrt(1 - zeta^2)) * 100;
    
    % Calculate Settling Time (2% Criterion)
    ts = 4 / (zeta * wn);
    
    fprintf('Peak Time (tp): %.4f seconds\n', tp);
    fprintf('Percent Overshoot (%%OS): %.2f%%\n', OS);
    fprintf('Settling Time (ts, 2%%): %.4f seconds\n', ts);
    
elseif zeta >= 1
    % For overdamped or critically damped systems, the 4/(zeta*wn) 
    % formula is less accurate, but we can still provide it.
    ts = 4 / (zeta * wn);
    fprintf('System is not underdamped. No OS or tp.\n');
    fprintf('Approximate Settling Time (ts): %.4f seconds\n', ts);
end