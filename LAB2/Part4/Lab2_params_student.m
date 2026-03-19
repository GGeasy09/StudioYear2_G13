clear;
% Pendulum Param
L = 0.1;     % [m]  (massless link)
mp = 0.05;   % [kg] (point mass)
g = 9.81;    % [m/s^2]
% Motor Param from experiment

J =0.0000126856024302855;
Lm= 0.002766949086;
R = 3.376477398;
b = 0.0000550590431422389;
ke = 0.0496487140384686;
 kt =0.04963704582;

%ke = 0.0528;
%kt = 0.0506;
%Lm = 0.0028445;
%R = 3.18;
%b = 7.7581E-05;
%J = 5.8559E-05;
Jarm = mp*L*L;
J_sum = J+Jarm;
A = J_sum*Lm;
B = J_sum*R+b*Lm;
C = R*b+ke*kt;
T = 0.5;
%0.49; %Tua Constant For Ref Feed Forward
T_dis = (Lm/R);

%Setpoint = 0;
%Initial_point = 0;
%num = kt;
%den = [ (Jsum*Lm), (Jsum*R + b*Lm), (b*R + kt*ke),0];

%sys_tf = tf(num, den);


%feed_forward = 1;