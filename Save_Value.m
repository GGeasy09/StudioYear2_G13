%% โหลดค่า Parameter
load('Lab1 Part2 Parameter\')

sim('MotorXploer.slx');

out = sim('MotorXploer.slx');

speed = out.Angular_Velocity.signals.values;
time = out.tout;
signal = out.Input_Signal_Real_Motor.signals.values;

figure;
hold on;
plot(time, speed);
plot(time, signal);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Motor Angular Velocity vs Time');
grid on;
hold off;
filename = 'Lab1 Part2 Data\lap1_ramp' % Save ชื่อไฟล์

save(filename, "speed","time");