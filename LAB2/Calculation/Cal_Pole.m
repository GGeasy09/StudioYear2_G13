run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

% compute poles (quadratic formula) and print them
disc = B*B - 4*A*C;
if disc < 0
    pole_pos1 = (-B + sqrt(complex(disc)))/(2*A);
    pole_pos2 = (-B - sqrt(complex(disc)))/(2*A);
else
    pole_pos1 = (-B + sqrt(disc))/(2*A);
    pole_pos2 = (-B - sqrt(disc))/(2*A);
end
disp('pole_pos1 =')
disp(pole_pos1)
disp('pole_pos2 =')
disp(pole_pos2)

% Calculating Meetpoint using degrees
Meetpoint = 0.8226 * tand(53.76);
disp('Meetpont =')
disp(Meetpoint);

value = 1.6-1.68/tand(46.39);
disp(value);