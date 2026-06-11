%% 
% ==========================================================================
%  LAB3_HoldPosition_Analysis.m
%  Analyse what happens AFTER trajectory ends — how fast the motor settles
%  to within the acceptable error band, and which controller / feedforward
%  component drives the correction.
%
%  Post-trajectory hold logic (from LAB_Manager.c):
%    • Outer PID  → position error → velocity command
%    • Inner PID  → velocity error → voltage
%    • Reference FF  fed with 0 rad/s (no trajectory, keeps filter warm)
%    • Disturbance FF fed with Kalman X[2], decayed ×0.9/tick when settled
%    • Friction FF  capped at ±0.3 V
%    • Dead zone 0.12 deg  — outer PID = 0, integrals bleed ×0.95/tick
%
%  Signals used (LAB3 TX packet):
%    Kv_pos         [0]  Kalman position      [rad]
%    Kv_velo        [1]  Kalman velocity      [rad/s]
%    Kv_dist        [2]  Kalman disturbance
%    friction_ff    [3]  Friction feedforward [V]
%    traj_pos       [4]  Trajectory ref pos   [rad]
%    traj_velo      [5]  Trajectory ref vel   [rad/s]
%    control_effort [6]  Total vout           [V]
%    encoder_rad    [7]  Encoder position     [rad]
%    reference_ff   [8]  Reference FF output  [V]
%    disturbance_ff [9]  Disturbance FF output[V]
%    Trajectory_state[10] 1=complete/idle
% ==========================================================================
clear; clc; close all;

%% ---- Settings ----
dt              = 0.0005;   % 2 kHz
ACCEPT_DEG      = 0.10;     % acceptable error band [deg]
DEADZONE_DEG    = 0.12;     % controller dead zone  [deg]
T_PRE           = 0.3;      % s before traj end to show
T_POST_PAD      = 1.0;      % extra seconds to show AFTER entering dead zone
T_POST_MAX      = 120.0;    % hard cap (in case it never enters dead zone)

%% ---- Load data ----
[file, path] = uigetfile('*.mat', 'Select LAB3 data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp  = load(fullfile(path, file));
vars = fieldnames(tmp);

enc_deg=[]; kv_pos=[]; kv_velo=[]; kv_dist=[];
ctrl=[]; ref_ff=[]; dist_ff=[]; friction_ff=[];
traj_complete=[]; traj_deg=[]; t=[];

for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v,'Simulink.SimulationData.Dataset')
        try
            kv_pos        = double(v.getElement('Kv_pos').Values.Data(:));
            kv_velo       = double(v.getElement('Kv_velo').Values.Data(:));
            kv_dist       = double(v.getElement('Kv_dist').Values.Data(:));
            friction_ff   = double(v.getElement('friction_ff').Values.Data(:));
            ctrl          = double(v.getElement('control effort').Values.Data(:));
            ref_ff        = double(v.getElement('reference_ff').Values.Data(:));
            dist_ff       = double(v.getElement('disturbance_ff').Values.Data(:));
            traj_complete = double(v.getElement('Trajectory_state').Values.Data(:));
            traj_deg      = double(v.getElement('traj_pos_degree').Values.Data(:));
            enc_deg       = double(v.getElement('encoder_pos_degree').Values.Data(:));
            t             = double(v.getElement('encoder_pos_degree').Values.Time(:));
        catch e
            error('Signal extraction failed: %s', e.message);
        end
        break;
    end
end
if isempty(enc_deg); error('No dataset found.'); end
N = length(enc_deg);
if isempty(t); t = (0:N-1)'*dt; end
fprintf('Loaded %d samples (%.2f s)\n', N, t(end));

%% ---- Detect trajectory end ----
traj_d  = diff(traj_complete);
falling = find(traj_d < -0.5);   % 1→0 : traj starts
rising  = find(traj_d >  0.5);   % 0→1 : traj ends

if isempty(falling); error('Trajectory never ran.'); end
i_traj_start = falling(end) + 1;

rising_after = rising(rising > i_traj_start);
if isempty(rising_after)
    warning('Trajectory never completed — using end of data.');
    i_traj_end = N;
else
    i_traj_end = rising_after(1) + 1;
end

target_deg = traj_deg(i_traj_end);
fprintf('Trajectory end : t = %.3f s\n', t(i_traj_end));
fprintf('Target         : %.4f deg\n\n', target_deg);

%% ---- Define hold window ----
%  Extend until error stays inside ±DEADZONE_DEG continuously,
%  then add T_POST_PAD seconds of padding.
i_hold_start = i_traj_end;

% find last sample OUTSIDE the dead zone
pos_err_deg_tmp = enc_deg - (traj_deg(i_traj_end));   % temp, target not yet set
i_max_search    = min(N, i_traj_end + round(T_POST_MAX/dt));
outside_dz      = find(abs(pos_err_deg_tmp(i_hold_start:i_max_search)) > DEADZONE_DEG);

if isempty(outside_dz)
    % already inside dead zone at traj end — just show padding
    i_dz_entry = i_hold_start;
else
    i_dz_entry = i_hold_start + outside_dz(end);   % last tick outside DZ
end

i_hold_end  = min(N, i_dz_entry + round(T_POST_PAD/dt));
i_pre_start = max(1, i_traj_end - round(T_PRE/dt));
fprintf('Dead zone entry: t = %.3f s after traj end\n', (i_dz_entry - i_traj_end)*dt);

i_hold  = i_hold_start:i_hold_end;
t_hold  = t(i_hold) - t(i_traj_end);   % t=0 at hold start
t_win   = t(i_pre_start:i_hold_end) - t(i_traj_end);

%% ---- Position error during hold ----
pos_err_deg = enc_deg - target_deg;   % signed error [deg]

%% ---- Decompose control effort ----
pid_component = ctrl - ref_ff - dist_ff;   % inner+outer PID only

%% ---- Settling time to ACCEPT_DEG ----
unsettled = find(abs(pos_err_deg(i_hold)) > ACCEPT_DEG);
if isempty(unsettled)
    settle_time = 0;
    fprintf('Already within %.2f deg at trajectory end.\n', ACCEPT_DEG);
else
    settle_time = (unsettled(end) - 1) * dt;
    fprintf('Settling time (±%.2f deg) : %.4f s\n', ACCEPT_DEG, settle_time);
end

%% ---- RMS contribution per component (during hold) ----
rms_pid    = rms(pid_component(i_hold));
rms_ref    = rms(ref_ff(i_hold));
rms_dist   = rms(dist_ff(i_hold));
rms_frict  = rms(friction_ff(i_hold));
rms_total  = rms(ctrl(i_hold));

fprintf('\n--- Control Effort RMS during hold ---\n');
fprintf('  Total        : %.5f V\n', rms_total);
fprintf('  PID only     : %.5f V  (%.1f%%)\n', rms_pid,   rms_pid/rms_total*100);
fprintf('  Reference FF : %.5f V  (%.1f%%)\n', rms_ref,   rms_ref/rms_total*100);
fprintf('  Disturbance FF: %.5f V (%.1f%%)\n', rms_dist,  rms_dist/rms_total*100);
fprintf('  Friction FF  : %.5f V  (%.1f%%)\n', rms_frict, rms_frict/rms_total*100);

%% ---- Final steady-state error ----
ss_idx     = i_hold(round(0.8*numel(i_hold)):end);
ss_err_deg = mean(pos_err_deg(ss_idx));
fprintf('\n  Final SS error : %.5f deg\n', ss_err_deg);
fprintf('  Max |err| hold : %.5f deg\n\n', max(abs(pos_err_deg(i_hold))));

%% ================================================================
%  FIGURES
%% ================================================================
win = i_pre_start:i_hold_end;

%% ---- Figure 1: Position error + settle band ----
figure('Name','Hold Position — Error','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.55 0.55 0.38]);

plot(t_win, pos_err_deg(win), 'b', 'LineWidth', 1.2); hold on;
yline( ACCEPT_DEG,  'g--', sprintf('+%.2f°', ACCEPT_DEG),  'LineWidth',1.0,'LabelHorizontalAlignment','left');
yline(-ACCEPT_DEG,  'g--', sprintf('-%.2f°', ACCEPT_DEG),  'LineWidth',1.0,'LabelHorizontalAlignment','left');
yline( DEADZONE_DEG,'r:',  sprintf('+%.2f° dead zone',DEADZONE_DEG),'LineWidth',0.8,'LabelHorizontalAlignment','left');
yline(-DEADZONE_DEG,'r:',  sprintf('-%.2f°',DEADZONE_DEG), 'LineWidth',0.8,'LabelHorizontalAlignment','left');
yline(0, 'k-', 'LineWidth', 0.5);
xline(0, 'k--', 'Traj end', 'LineWidth', 1.0);

if settle_time > 0
    xline(settle_time, 'm-', sprintf('Settled %.3f s', settle_time), ...
        'LineWidth', 1.2, 'LabelHorizontalAlignment', 'right');
end
dz_time = (i_dz_entry - i_traj_end) * dt;
xline(dz_time, 'Color',[0.85 0.33 0.1], 'LineStyle','--', ...
    'Label', sprintf('DZ entry %.3f s', dz_time), ...
    'LineWidth', 1.0, 'LabelHorizontalAlignment', 'right');

xlabel('Time from traj end [s]'); ylabel('Position error [deg]');
title(sprintf('Hold Position Error  |  Settle=%.4f s  SS err=%.5f°  Max=%.4f°', ...
    settle_time, ss_err_deg, max(abs(pos_err_deg(i_hold)))));
grid on;

%% ---- Figure 2: Control effort decomposition ----
figure('Name','Hold Position — Control Effort','NumberTitle','off',...
    'Units','normalized','Position',[0.57 0.55 0.42 0.38]);

subplot(2,1,1);
plot(t_win, ctrl(win),            'b',  'LineWidth',1.3, 'DisplayName','Total'); hold on;
plot(t_win, pid_component(win),   'r',  'LineWidth',1.0, 'DisplayName','PID');
plot(t_win, ref_ff(win),          'm',  'LineWidth',1.0, 'DisplayName','Ref FF');
plot(t_win, dist_ff(win),         'g',  'LineWidth',1.0, 'DisplayName','Dist FF');
plot(t_win, friction_ff(win),     'Color',[0.8 0.5 0], 'LineWidth',1.0, 'DisplayName','Friction FF');
xline(0,'k--','LineWidth',1.0,'HandleVisibility','off');
yline(0,'k-', 'LineWidth',0.5,'HandleVisibility','off');
grid on; ylabel('Voltage [V]'); legend('Location','best');
title('Control Effort Breakdown');

subplot(2,1,2);
contrib = [rms_pid, rms_ref, rms_dist, rms_frict];
b = bar(contrib, 'FaceColor','flat');
b.CData = [0.8 0.2 0.2; 0.8 0.2 0.8; 0.2 0.8 0.2; 0.8 0.5 0.0];
set(gca,'XTickLabel',{'PID','Ref FF','Dist FF','Friction FF'});
ylabel('RMS [V]'); title('Average Contribution (RMS during hold)'); grid on;
for k = 1:4
    pct = contrib(k) / rms_total * 100;
    text(k, contrib(k)+0.001, sprintf('%.3fV\n(%.1f%%)', contrib(k), pct), ...
        'HorizontalAlignment','center','FontSize',8);
end
xlabel('Time from traj end [s]');

%% ---- Figure 3: Velocity and Disturbance decay ----
figure('Name','Hold Position — Kalman States','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.08 0.55 0.40]);

subplot(2,1,1);
plot(t_win, kv_velo(win), 'b', 'LineWidth',1.2);
xline(0,'k--','LineWidth',1.0);
yline(0,'k-','LineWidth',0.5);
grid on; ylabel('Velocity [rad/s]');
title('Kalman Velocity (should decay to 0 at rest)');

subplot(2,1,2);
plot(t_win, kv_dist(win), 'r', 'LineWidth',1.2); hold on;
plot(t_win, dist_ff(win), 'g--', 'LineWidth',1.0, 'DisplayName','Dist FF output');
xline(0,'k--','LineWidth',1.0,'HandleVisibility','off');
yline(0,'k-','LineWidth',0.5,'HandleVisibility','off');
legend('Kalman X[2]','Dist FF output','Location','best');
grid on; ylabel('[Nm] / [V]'); xlabel('Time from traj end [s]');
title('Disturbance Estimate and FF (decays ×0.9/tick when |err| < 0.12°)');

%% ---- Figure 4: Error envelope zoom ----
figure('Name','Hold Position — Zoom','NumberTitle','off',...
    'Units','normalized','Position',[0.57 0.08 0.42 0.40]);

T_ZOOM = min((i_hold_end - i_traj_end)*dt, settle_time * 2 + 0.5);
i_zoom = i_hold(t(i_hold)-t(i_traj_end) <= T_ZOOM);
t_zoom = t(i_zoom) - t(i_traj_end);
err_zoom = pos_err_deg(i_zoom);

plot(t_zoom, err_zoom, 'b', 'LineWidth', 1.3); hold on;
yline( ACCEPT_DEG, 'g--', 'LineWidth', 1.2);
yline(-ACCEPT_DEG, 'g--', 'LineWidth', 1.2);
yline( DEADZONE_DEG, 'r:', 'LineWidth', 0.8);
yline(-DEADZONE_DEG, 'r:', 'LineWidth', 0.8);
yline(0,'k-','LineWidth',0.5);

if settle_time > 0 && settle_time <= T_ZOOM
    xline(settle_time,'m-','LineWidth',1.5);
    text(settle_time+0.02, ACCEPT_DEG*1.5, sprintf('%.3f s', settle_time), ...
        'Color','m','FontSize',9);
end
xlabel('Time from traj end [s]'); ylabel('Error [deg]');
title(sprintf('Zoom — settle to ±%.2f deg', ACCEPT_DEG));
grid on;
