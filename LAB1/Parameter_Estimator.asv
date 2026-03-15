clear();
filename = 'Save_Variable\Stairwait2.mat';
load(filename);
if exist('moter_L','var')
    motor_L = moter_L;
    clear moter_L
    save(filename,'-append')
end
motor_B = 5.51E-05;
motor_Eff = 0.9997649845;
motor_J = 1.27E-05;
motor_Ke = 0.04964871404;
velocity = double(velocity);
% save(filename)