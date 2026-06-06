% ==========================================================================
%  LAB3_Cascade_Analysis.m
%  Analyse full cascade position control performance.
%
%  Signals (Simulink log):
%   Kv_pos            — Kalman position estimate    [rad]
%   Kv_velo           — Kalman velocity estimate    [rad/s]
%   Kv_dist           — Kalman disturbance estimate [Nm]
%   Kv_current        — Kalman current estimate     [A]
%   traj_pos          — trajectory reference pos    [rad]
%   traj_velo         — trajectory reference vel    [rad/s]
%   control effort    — total voltage output        [V]
%   encoder_rad       — actual encoder position     [rad]
%   reference_ff      — reference feedforward       [V]
%   disturbance_ff    — disturbance feedforward     [V]
%   Trajectory_state  — 1 = trajectory complete
%   encoder_pos_degree — encoder position           [deg]
%   traj_pos_degree   — trajectory reference pos    [deg]
%
%  Metrics:
%   Position tracking error (RMS, max, final)
%   Velocity tracking error (RMS, max)
%   Overshoot [deg]
%   Settling time [s]  (±settle_band deg from target)
%   Control effort RMS & peak
%   FF contribution ratio
% ==========================================================================
clear; clc; close all;

%% ---- Settings ---- (edit here) ----
dt           = 0.0005;    % 2 kHz
settle_band  = 1.0;       % deg  — settling criterion (±X deg)
T_PRE        = 0.5;       % s before trajectory start to show
T_POST       = 2.0;       % s after trajectory end to show

%% ---- Load Data ----
[file, path] = uigetfile('*.mat', 'Select LAB3 cascade data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp  = load(fullfile(path, file));
vars = fieldnames(tmp);

% Signal containers
enc_rad = []; enc_deg = []; kv_pos = []; kv_velo = []; kv_dist = []; friction_ff_log = [];
traj_pos = []; traj_velo = []; ctrl = []; ref_ff = []; dist_ff = [];
traj_complete = []; traj_deg = []; t = [];

for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v, 'Simulink.SimulationData.Dataset')
        try
            enc_rad       = double(v.getElement('encoder_rad').Values.Data(:));
            enc_deg       = double(v.getElement('encoder_pos_degree').Values.Data(:));
            kv_pos        = double(v.getElement('Kv_pos').Values.Data(:));
            kv_velo       = double(v.getElement('Kv_velo').Values.Data(:));
            kv_dist       = double(v.getElement('Kv_dist').Values.Data(:));
            friction_ff_log = double(v.getElement('friction_ff').Values.Data(:));
            traj_pos      = double(v.getElement('traj_pos').Values.Data(:));
            traj_velo     = double(v.getElement('traj_velo').Values.Data(:));
            ctrl          = double(v.getElement('control effort').Values.Data(:));
            ref_ff        = double(v.getElement('reference_ff').Values.Data(:));
            dist_ff       = double(v.getElement('disturbance_ff').Values.Data(:));
            traj_complete = double(v.getElement('Trajectory_state').Values.Data(:));
            traj_deg      = double(v.getElement('traj_pos_degree').Values.Data(:));
            t             = double(v.getElement('encoder_rad').Values.Time(:));
        catch e
            error('Signal extraction failed: %s\nCheck signal names match the Simulink log.', e.message);
        end
        break;
    end
end

if isempty(enc_rad)
    error('No Simulink.SimulationData.Dataset found in file.');
end

N = length(enc_rad);
if isempty(t); t = (0:N-1)' * dt; end
fprintf('Loaded %d samples (%.2f s)\n', N, t(end));

%% ---- Find trajectory window ----
% Trajectory_state: 1 = idle/complete,  0 = running
% Detect the 1→0 falling edge (trajectory begins) then 0→1 rising edge (trajectory ends).
traj_d = diff(traj_complete);
falling = find(traj_d < -0.5);   % 1→0 transitions
rising  = find(traj_d >  0.5);   % 0→1 transitions

if isempty(falling)
    error('Trajectory never ran (Trajectory_state never went to 0). Check data.');
end

% Use the LAST falling edge (most recent trigger)
traj_started = falling(end) + 1;

% Find the first rising edge AFTER that falling edge
rising_after = rising(rising > traj_started);
if isempty(rising_after)
    warning('Trajectory_state never returned to 1 — trajectory may not have completed. Using end of data.');
    traj_ended = N;
else
    traj_ended = rising_after(1) + 1;
end

% Target position = traj_pos at the moment trajectory completed
target_deg = traj_deg(traj_ended);
target_rad = traj_pos(traj_ended);

fprintf('Trajectory start : t = %.3f s\n', t(traj_started));
fprintf('Trajectory end   : t = %.3f s\n', t(traj_ended));
fprintf('Target position  : %.2f deg (%.4f rad)\n\n', target_deg, target_rad);

% Plot window indices
i_win_start = max(1, traj_started - round(T_PRE  / dt));
i_win_end   = min(N, traj_ended   + round(T_POST / dt));
t_win       = t(i_win_start:i_win_end) - t(traj_started);   % t=0 at traj start
t0_offset   = t(traj_started);

% Trajectory and settling segments
i_traj   = traj_started:traj_ended;
i_settle = traj_ended:i_win_end;

%% ---- Compute tracking errors ----
pos_err_deg = traj_deg - enc_deg;     % trajectory ref − actual  [deg]
pos_err_rad = traj_pos - enc_rad;     % [rad]
vel_err     = traj_velo - kv_velo;    % [rad/s]

%% ---- Metrics ----
% --- During trajectory ---
rms_pos_err  = rms(pos_err_deg(i_traj));
max_pos_err  = max(abs(pos_err_deg(i_traj)));
rms_vel_err  = rms(vel_err(i_traj));
max_vel_err  = max(abs(vel_err(i_traj)));

% --- Overshoot: percentage of total trajectory travel ---
traj_amplitude = abs(target_deg - traj_deg(traj_started));
enc_post = enc_deg(i_settle);
if target_deg > traj_deg(traj_started)
    overshoot_deg = max(0, max(enc_post) - target_deg);
else
    overshoot_deg = max(0, target_deg - min(enc_post));
end
overshoot = overshoot_deg / traj_amplitude * 100;   % [%]

% --- Final position error ---
ss_start   = max(1, round(0.8 * numel(i_settle)));
final_pos  = mean(enc_deg(i_settle(ss_start:end)));
final_err  = target_deg - final_pos;

% --- Settling time (from traj_ended) ---
band = settle_band;
unsettled = find(abs(enc_deg(i_settle) - target_deg) > band);
if isempty(unsettled)
    settle_time = 0;
else
    settle_time = (unsettled(end) - 1) * dt;
end

% --- Control effort ---
rms_ctrl  = rms(ctrl(i_traj));
peak_ctrl = max(abs(ctrl(i_traj)));

% --- Feedforward contribution ratio during trajectory ---
pid_component  = ctrl - ref_ff - dist_ff;
rms_pid        = rms(pid_component(i_traj));
rms_ref_ff     = rms(ref_ff(i_traj));
rms_dist_ff    = rms(dist_ff(i_traj));
ff_ratio       = (rms_ref_ff + rms_dist_ff) / (rms_ctrl + 1e-9) * 100;

%% ---- Print results ----
fprintf('===========================================================\n');
fprintf('  LAB3 CASCADE CONTROL — PERFORMANCE METRICS\n');
fprintf('===========================================================\n');
fprintf('  TARGET POSITION    : %.2f deg\n', target_deg);
fprintf('-----------------------------------------------------------\n');
fprintf('  POSITION TRACKING (during trajectory)\n');
fprintf('    RMS error        : %.4f deg\n', rms_pos_err);
fprintf('    Max error        : %.4f deg\n', max_pos_err);
fprintf('-----------------------------------------------------------\n');
fprintf('  VELOCITY TRACKING (during trajectory)\n');
fprintf('    RMS error        : %.4f rad/s\n', rms_vel_err);
fprintf('    Max error        : %.4f rad/s\n', max_vel_err);
fprintf('-----------------------------------------------------------\n');
fprintf('  ENDPOINT PERFORMANCE\n');
fprintf('    Overshoot        : %.4f %%  (%.4f deg / %.2f deg travel)\n', overshoot, overshoot_deg, traj_amplitude);
fprintf('    Final SS error   : %.4f deg\n', final_err);
fprintf('    Settling time    : %.4f s  (band = ±%.1f deg)\n', settle_time, settle_band);
fprintf('-----------------------------------------------------------\n');
fprintf('  CONTROL EFFORT\n');
fprintf('    RMS voltage      : %.4f V\n', rms_ctrl);
fprintf('    Peak voltage     : %.4f V\n', peak_ctrl);
fprintf('    FF ratio (RMS)   : %.1f %%  (ref+dist FF / total)\n', ff_ratio);
fprintf('    RMS ref FF       : %.4f V\n', rms_ref_ff);
fprintf('    RMS dist FF      : %.4f V\n', rms_dist_ff);
fprintf('    RMS PID only     : %.4f V\n', rms_pid);
fprintf('===========================================================\n\n');

%% ================================================================
%  FIGURES
%% ================================================================
win = i_win_start:i_win_end;   % shared index range

%% ---- Figure 1: Position Tracking ----
figure('Name','LAB3 — Position Tracking','NumberTitle','off',...
       'Units','normalized','Position',[0.01 0.53 0.48 0.42]);

subplot(2,1,1);
plot(t_win, traj_deg(win), 'k--', 'LineWidth',1.2, 'DisplayName','Trajectory ref'); hold on;
plot(t_win, enc_deg(win),  'b',   'LineWidth',1.2, 'DisplayName','Encoder actual');
plot(t_win, kv_pos(win)*57.2958, 'Color',[0 0.6 0], 'LineWidth',0.9, ...
     'DisplayName','Kalman pos');
xline(0,                    'r:',  'Traj start', 'LineWidth',1.0, 'HandleVisibility','off');
xline(t(traj_ended)-t0_offset, 'm:', 'Traj end',  'LineWidth',1.0, 'HandleVisibility','off');
yline(target_deg, 'r--', 'Target', 'LineWidth',0.8, 'HandleVisibility','off');
grid on; ylabel('Position [deg]'); legend('Location','best');
title(sprintf('Position Tracking   (Overshoot=%.3f%%  SSe=%.3f°  Settle=%.3fs)', ...
      overshoot, final_err, settle_time));

subplot(2,1,2);
plot(t_win, pos_err_deg(win), 'r', 'LineWidth',1.2);
xline(0, 'r:', 'LineWidth',1.0);
xline(t(traj_ended)-t0_offset, 'm:', 'LineWidth',1.0);
yline( settle_band, 'k--', 'LineWidth',0.8);
yline(-settle_band, 'k--', sprintf('±%.1f°', settle_band), 'LineWidth',0.8);
yline(0, 'k-', 'LineWidth',0.5);
grid on; ylabel('Error [deg]'); xlabel('Time [s]');
title(sprintf('Position Error  |  RMS=%.4f°   Max=%.4f°', rms_pos_err, max_pos_err));

%% ---- Figure 2: Velocity Tracking ----
figure('Name','LAB3 — Velocity Tracking','NumberTitle','off',...
       'Units','normalized','Position',[0.51 0.53 0.48 0.42]);

subplot(2,1,1);
plot(t_win, traj_velo(win), 'k--', 'LineWidth',1.2, 'DisplayName','Trajectory ref'); hold on;
plot(t_win, kv_velo(win),   'b',   'LineWidth',1.2, 'DisplayName','Kalman velocity');
xline(0, 'r:', 'LineWidth',1.0, 'HandleVisibility','off');
xline(t(traj_ended)-t0_offset, 'm:', 'LineWidth',1.0, 'HandleVisibility','off');
grid on; ylabel('Velocity [rad/s]'); legend('Location','best');
title(sprintf('Velocity Tracking   RMS err=%.4f rad/s   Max err=%.4f rad/s', ...
      rms_vel_err, max_vel_err));

subplot(2,1,2);
plot(t_win, vel_err(win), 'r', 'LineWidth',1.2);
xline(0, 'r:', 'LineWidth',1.0);
xline(t(traj_ended)-t0_offset, 'm:', 'LineWidth',1.0);
yline(0, 'k-', 'LineWidth',0.5);
grid on; ylabel('Error [rad/s]'); xlabel('Time [s]');
title('Velocity Tracking Error  (ref − Kalman)');

%% ---- Figure 3: Control Effort Breakdown ----
figure('Name','LAB3 — Control Effort','NumberTitle','off',...
       'Units','normalized','Position',[0.01 0.05 0.65 0.42]);

subplot(2,1,1);
plot(t_win, ctrl(win),                        'b',  'LineWidth',1.3, 'DisplayName','Total output');  hold on;
plot(t_win, pid_component(win),               'r',  'LineWidth',1.0, 'DisplayName','PID only');
plot(t_win, ref_ff(win),                      'm',  'LineWidth',1.0, 'DisplayName','Reference FF');
plot(t_win, dist_ff(win),                     'g',  'LineWidth',1.0, 'DisplayName','Disturbance FF');
xline(0, 'k:', 'LineWidth',1.0, 'HandleVisibility','off');
xline(t(traj_ended)-t0_offset, 'k:', 'LineWidth',1.0, 'HandleVisibility','off');
yline(0, 'k-', 'LineWidth',0.5, 'HandleVisibility','off');
grid on; ylabel('Voltage [V]'); legend('Location','best');
title(sprintf('Control Effort  |  FF ratio=%.1f%%   RMS=%.3fV   Peak=%.3fV', ...
      ff_ratio, rms_ctrl, peak_ctrl));

subplot(2,1,2);
% Stacked absolute contribution bar (average during trajectory)
contrib = [rms_pid, rms_ref_ff, rms_dist_ff];
bar(contrib, 'FaceColor','flat', 'CData', [0.2 0.4 0.8; 0.8 0.2 0.8; 0.2 0.8 0.2]);
set(gca, 'XTickLabel', {'PID','Ref FF','Dist FF'});
ylabel('RMS Voltage [V]'); title('Average Contribution (RMS during trajectory)'); grid on;
for k = 1:3
    text(k, contrib(k) + 0.02, sprintf('%.3fV', contrib(k)), ...
         'HorizontalAlignment','center', 'FontSize',9);
end

%% ---- Figure 4: Kalman States ----
figure('Name','LAB3 — Kalman States','NumberTitle','off',...
       'Units','normalized','Position',[0.67 0.05 0.32 0.42]);

subplot(2,1,1);
plot(t_win, kv_dist(win), 'b', 'LineWidth',1.2);
xline(0, 'k:', 'LineWidth',1.0);
xline(t(traj_ended)-t0_offset, 'k:', 'LineWidth',1.0);
grid on; ylabel('\tau_{dist} [Nm]'); xlabel('Time [s]');
title('Kalman Disturbance Estimate (X[2])');

subplot(2,1,2);
plot(t_win, friction_ff_log(win), 'r', 'LineWidth',1.2);
xline(0, 'k:', 'LineWidth',1.0);
xline(t(traj_ended)-t0_offset, 'k:', 'LineWidth',1.0);
grid on; ylabel('Voltage [V]'); xlabel('Time [s]');
title('Friction Feedforward');
