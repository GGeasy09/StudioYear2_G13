% ==========================================================================
%  LAB3_Trapezoid_Sim.m
%  Simulate and visualise the trapezoidal velocity profile.
%
%  Profile shape:
%   Phase 1: Accelerate from 0 → v_max  (at rate a_max)
%   Phase 2: Cruise at v_max
%   Phase 3: Decelerate from v_max → 0  (at rate a_max)
%
%  Edit parameters below to match lab_config.h settings.
% ==========================================================================
clear; clc; close all;

%% ---- Parameters (match lab_config.h) ----
v_max      = 4.05;    % CFG_TRAP_VMAX  [rad/s]
a_max      = 4.8;     % CFG_TRAP_AMAX  [rad/s²]
target_deg = 360.0;   % CFG_POS_SETPOINT_DEG [deg]
target_rad = target_deg * pi / 180;

dt = 0.0005;          % 2 kHz (same as STM32 control loop)

%% ---- Compute phase durations ----
t_acc = v_max / a_max;                         % time to reach v_max
d_acc = 0.5 * v_max * t_acc;                  % distance covered during accel (and decel)

if 2 * d_acc > target_rad
    % Triangle profile: peak velocity is limited by distance
    warning('Target too short for full v_max. Switching to triangle profile.');
    v_peak = sqrt(a_max * target_rad);
    t_acc  = v_peak / a_max;
    d_acc  = 0.5 * v_peak * t_acc;
    t_cruise   = 0;
    d_cruise   = 0;
    v_max      = v_peak;
else
    d_cruise = target_rad - 2 * d_acc;
    t_cruise = d_cruise / v_max;
end

t_total = 2 * t_acc + t_cruise;

fprintf('===== Trapezoid Profile Summary =====\n');
fprintf('  Target      : %.2f deg  (%.4f rad)\n', target_deg, target_rad);
fprintf('  v_max       : %.4f rad/s\n', v_max);
fprintf('  a_max       : %.4f rad/s²\n', a_max);
fprintf('  t_accel     : %.4f s\n', t_acc);
fprintf('  t_cruise    : %.4f s\n', t_cruise);
fprintf('  t_decel     : %.4f s\n', t_acc);
fprintf('  t_total     : %.4f s\n', t_total);
fprintf('=====================================\n\n');

%% ---- Build time vector and profiles ----
t = (0:dt:t_total)';
N = length(t);

velocity    = zeros(N, 1);
position    = zeros(N, 1);
accel_prof  = zeros(N, 1);

t1 = t_acc;
t2 = t_acc + t_cruise;
t3 = t_total;

for k = 1:N
    tk = t(k);
    if tk <= t1
        % Phase 1: Accelerate
        velocity(k)   = a_max * tk;
        position(k)   = 0.5 * a_max * tk^2;
        accel_prof(k) = a_max;
    elseif tk <= t2
        % Phase 2: Cruise
        velocity(k)   = v_max;
        position(k)   = d_acc + v_max * (tk - t1);
        accel_prof(k) = 0;
    else
        % Phase 3: Decelerate
        td = tk - t2;
        velocity(k)   = v_max - a_max * td;
        position(k)   = d_acc + d_cruise + v_max * td - 0.5 * a_max * td^2;
        accel_prof(k) = -a_max;
    end
end

% Clamp small floating-point negatives
velocity(velocity < 0) = 0;
position(position > target_rad) = target_rad;

position_deg = position * 180 / pi;

%% ---- Figure 1: Three-panel profile ----
figure('Name','Trapezoid Profile Simulation','NumberTitle','off',...
    'Units','normalized','Position',[0.05 0.1 0.55 0.75]);

% --- Velocity ---
subplot(3,1,1);
plot(t, velocity, 'b', 'LineWidth', 2);
hold on;
% Shade phases
patch([0 t1 t1 0], [0 0 v_max*1.1 v_max*1.1], [0.85 0.93 1.0], ...
    'EdgeColor','none', 'FaceAlpha', 0.4);
patch([t1 t2 t2 t1], [0 0 v_max*1.1 v_max*1.1], [0.85 1.0 0.85], ...
    'EdgeColor','none', 'FaceAlpha', 0.4);
patch([t2 t3 t3 t2], [0 0 v_max*1.1 v_max*1.1], [1.0 0.9 0.85], ...
    'EdgeColor','none', 'FaceAlpha', 0.4);
xline(t1, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
xline(t2, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
yline(v_max, 'r--', sprintf('v_{max}=%.2f rad/s', v_max), ...
    'LineWidth', 0.8, 'LabelHorizontalAlignment','left');
ylim([0, v_max * 1.15]);
ylabel('Velocity [rad/s]');
title(sprintf('Trapezoidal Velocity Profile  —  Target: %.0f°   v_{max}: %.2f rad/s   a_{max}: %.1f rad/s²', ...
    target_deg, v_max, a_max));
legend('Velocity', 'Accel', 'Cruise', 'Decel', 'Location','south','Orientation','horizontal');
grid on;

% Annotate phase times
text(t1/2,      v_max*0.5, sprintf('Accel\n%.3f s', t_acc), ...
    'HorizontalAlignment','center', 'FontSize',9, 'Color',[0.1 0.3 0.7]);
text((t1+t2)/2, v_max*0.5, sprintf('Cruise\n%.3f s', t_cruise), ...
    'HorizontalAlignment','center', 'FontSize',9, 'Color',[0.1 0.5 0.1]);
text((t2+t3)/2, v_max*0.5, sprintf('Decel\n%.3f s', t_acc), ...
    'HorizontalAlignment','center', 'FontSize',9, 'Color',[0.6 0.2 0.1]);

% --- Position ---
subplot(3,1,2);
plot(t, position_deg, 'Color',[0 0.55 0], 'LineWidth', 2);
hold on;
xline(t1, 'k--', 'LineWidth', 0.8);
xline(t2, 'k--', 'LineWidth', 0.8);
yline(target_deg, 'r--', sprintf('%.0f°', target_deg), ...
    'LineWidth', 0.8, 'LabelHorizontalAlignment','left');
ylabel('Position [deg]');
grid on;
title(sprintf('Position  (final = %.4f°)', position_deg(end)));

% --- Acceleration ---
subplot(3,1,3);
plot(t, accel_prof, 'Color',[0.8 0.2 0.2], 'LineWidth', 2);
hold on;
xline(t1, 'k--', 'LineWidth', 0.8);
xline(t2, 'k--', 'LineWidth', 0.8);
yline( a_max, 'b--', sprintf('+%.1f', a_max), 'LabelHorizontalAlignment','left', 'LineWidth',0.8);
yline(-a_max, 'b--', sprintf('−%.1f', a_max), 'LabelHorizontalAlignment','left', 'LineWidth',0.8);
yline(0, 'k-', 'LineWidth', 0.5);
ylim([-a_max*1.4, a_max*1.4]);
ylabel('Acceleration [rad/s²]');
xlabel('Time [s]');
grid on;
title('Acceleration Profile');

sgtitle(sprintf('Trapezoid Sim  |  t_{total}=%.3f s  |  %.0f° → %.4f°', ...
    t_total, target_deg, position_deg(end)), 'FontWeight','bold');

%% ---- Figure 2: Phase diagram (velocity vs position) ----
figure('Name','Phase Diagram — v vs θ','NumberTitle','off',...
    'Units','normalized','Position',[0.62 0.1 0.35 0.42]);

i1 = t <= t1;
i2 = (t > t1) & (t <= t2);
i3 = t > t2;

plot(position_deg(i1), velocity(i1), 'b',  'LineWidth', 2, 'DisplayName','Accel');  hold on;
plot(position_deg(i2), velocity(i2), 'g',  'LineWidth', 2, 'DisplayName','Cruise');
plot(position_deg(i3), velocity(i3), 'r',  'LineWidth', 2, 'DisplayName','Decel');
plot(0,          0,      'ko', 'MarkerFaceColor','k', 'MarkerSize',6, 'HandleVisibility','off');
plot(target_deg, 0,      'rs', 'MarkerFaceColor','r', 'MarkerSize',6, 'HandleVisibility','off');
xlabel('Position [deg]');
ylabel('Velocity [rad/s]');
title('Phase Diagram  (v vs θ)');
legend('Location','north');
grid on;
ylim([0, v_max * 1.15]);
