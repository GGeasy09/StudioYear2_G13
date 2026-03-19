clear;
% Pendulum Param
L = 0.1;     % [m]  (massless link)
mp = 0.05;   % [kg] (point mass)
g = 9.81;    % [m/s^2]
% Motor Param from experiment
ke = 0.0528;
kt = 0.0506;
Lm = 0.0028445;
R = 3.18;
b = 7.7581E-05;
J = 5.8559E-05;
Jarm = mp*L*L;
J_sum = J+Jarm;
A = J_sum*Lm;
B = J_sum*R+b*Lm;
C = R*b+ke*kt;
T = 0.0001; %Tua Constant For Feed Forward

Setpoint = 0;
Initial_point = 0;
num = kt;
den = [ (J_sum*Lm), (J_sum*R + b*Lm), (b*R + kt*ke),0];

sys_tf = tf(num, den);


feed_forward = 1;