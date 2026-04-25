%% Root Locus Zero Placement Calculator
clear;

run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

% 1. Inputs
Design_Point = -1.6 + 1.68j;
    SetofPole = [0,0,-1116.5,-1.1]; % <--- Add your actual poles here

% 2. Calculate Required Zero (Angle Condition)
pole_angles = atan2d(imag(Design_Point) - imag(SetofPole), ...
                     real(Design_Point) - real(SetofPole));
                 
sum_theta_poles = sum(pole_angles);
theta_z_req = sum_theta_poles - 180; % Angle needed from the zero
z_loc = real(Design_Point) - (imag(Design_Point) / tand(theta_z_req));

% 3. Plotting
figure('Color', 'w');
hold on; grid on; axis equal;

% Plot Design Point
plot(real(Design_Point), imag(Design_Point), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12);
text(real(Design_Point)+0.1, imag(Design_Point)+0.1, 'Design Point (s_d)', 'FontSize', 10, 'FontWeight', 'bold');

% Plot Poles and draw lines to Design Point
for i = 1:length(SetofPole)
    % Plot Pole
    plot(real(SetofPole(i)), imag(SetofPole(i)), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
    
    % Draw Line to Design Point
    line([real(SetofPole(i)) real(Design_Point)], [imag(SetofPole(i)) imag(Design_Point)], ...
        'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.2);
    
    text(real(SetofPole(i)), -0.3, sprintf('P_%d', i), 'Color', 'r', 'HorizontalAlignment', 'center');
end

% Plot Calculated Zero and draw line to Design Point
plot(z_loc, 0, 'go', 'LineWidth', 2, 'MarkerSize', 10);
line([z_loc real(Design_Point)], [0 imag(Design_Point)], ...
    'Color', 'g', 'LineStyle', '-', 'LineWidth', 1.5);
text(z_loc, -0.3, 'Zero (z_o)', 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');

% Aesthetics
xlabel('Real Axis (\sigma)');
ylabel('Imaginary Axis (j\omega)');
title('Angle Condition Geometry: Vectors to Design Point');
legend('Design Point', 'Poles', 'Pole-to-DP Vector', 'Calculated Zero', 'Zero-to-DP Vector');

% Reference Real Axis Line
line([min([SetofPole, z_loc])-1, 1], [0 0], 'Color', 'k', 'LineWidth', 0.5);

% 3. Apply Angle Condition: Angle(Zeros) - Angle(Poles) = -180 (or 180)
% For a single zero: Theta_z - sum_theta_poles = -180
% Theta_z = sum_theta_poles - 180
theta_z = sum_theta_poles - 180;

% Normalize angle to be within [0, 360] or [-180, 180] if needed
theta_z = mod(theta_z, 360);

% 4. Calculate Zero Position (z_loc) on the Real Axis
% Geometry: tan(theta_z) = imag(Design_Point) / (real(Design_Point) - z_loc)
% Therefore: z_loc = real(Design_Point) - [imag(Design_Point) / tand(theta_z)]

z_loc = real(Design_Point) - (imag(Design_Point) / tand(theta_z));

% 5. Display Results
fprintf('Sum of Pole Angles: %.2f degrees\n', sum_theta_poles);
fprintf('Required Zero Angle (theta_z): %.2f degrees\n', theta_z);
fprintf('-------------------------------------------\n');
fprintf('Calculated Zero Position: %.4f\n', z_loc);

%%finding Magnitude
s = Design_Point;
k_equa = s*(A*s*s+B*s+C)/(s+2.5597)/kt;

% 6. Calculate the magnitude of the transfer function at the Design Point
magnitude = abs(k_equa);
fprintf('Magnitude of the transfer function at the Design Point: %.4f\n', magnitude);

