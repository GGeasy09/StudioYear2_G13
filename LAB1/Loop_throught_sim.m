clear();
load('Save_Variable - Copy\Ramp0.1.mat')
motor_B = 0.0001097471783;
motor_Eff = 0.9994969307;
motor_J = 1.13E-05;
motor_Ke = 0.04336937833;
% motor_B = 1.02E-04;
% motor_Eff = 0.9997684157;
% motor_J = 0.0002886642529;
% motor_Ke = 0.0439752384;
nigga = sim("Lab1_parameter_estimation_student.slx");

predict_signal = nigga.Estimate_Result.signals.values;
time_predict = nigga.tout;

figure;
hold on;
plot(time_predict,predict_signal,'LineWidth',2.5); % make predicted signal bolder
plot(time,velocity,'LineWidth',1.0);
xlabel('Time (s)');
ylabel('Velocity (units)');
legend('Predicted','Measured');
grid on;