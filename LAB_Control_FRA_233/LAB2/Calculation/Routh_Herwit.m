% Your parameters
Jarm = 0.05 * 0.1^2;
Lm = 0.0028445;
R = 3.18;
b = 7.7581E-05;
ke = 0.0528;
kt = 0.0506;

% Coefficients of the characteristic equation
a3 = Jarm * Lm;
a2 = Jarm * R + b * Lm;
a1 = b * R + ke * kt;

% Stability condition: a2*a1 - a3*(Kp*kt) > 0
% Solving for Kp_max:
Kp_max = (a2 * a1) / (a3 * kt);

fprintf('The system is stable for: 0 < Kp < %.4f\n', Kp_max);