clear;
%% save parameter
motor_R = 3.376;
moter_L = 0.00276;
% Other Input Parameter
Mode = 4; %step1;ramp2;chirp3;sine4;stair5
Step_Value = 0.5;%0-1
Ramp_Slope = 0.2;
Chirmp_Time = 40;
Chirmp_Freq = pi/2;
Sine_Freq = pi;
Stair_Wait = 2;
%% run file and check
sim('C:\Users\ACER\Documents\GitHub\StudioYear2_G13\MotorXploer.slx');
velocity = ans.Angular_Velocity.signals.values;
time = ans.tout;
signal = ans.Signal.signals.values;
save("Save_Variable\Sinepi.mat");
%% Plot Graph Check
figure;
hold on;
plot(time, velocity);
plot(time, signal);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity vs Time');
grid on;