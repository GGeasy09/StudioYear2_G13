% ==========================================================================
%  LAB4_DeadBand_Analysis.m
%  Detect motor dead band voltages from the 0→9V→0 ramp experiment.
%
%  Signals (Simulink log, 4 channels):
%   encoder_rad  — actual position [rad]          TX field 0
%   Kv_velo      — Kalman velocity [rad/s]         TX field 1
%   vout         — ramp voltage [V]                TX field 2
%   active flag  — 1=ramping, 0=idle              TX field 3
%
%  Results:
%   V_start  — voltage where encoder STARTS moving (ramp up)
%   V_stop   — voltage where encoder STOPS  moving (ramp down)
%   DeadBand — V_stop  (motor stops before it starts, hysteresis)
% ==========================================================================
clear; clc; close all;

%% ---- Settings ---- (edit here) ----
dt          = 0.0005;      % 2 kHz

% --- Detector 1: Kalman velocity threshold ---
VELO_THR    = 0.02;        % rad/s  — threshold to declare "moving"
SMOOTH_WIN  = 20;          % samples for velocity smoothing (noise reduction)

% --- Detector 2: Encoder position change threshold ---
ENC_WIN     = 50;          % samples to compute position delta over (25 ms window)
ENC_THR     = 0.002;       % rad — min position change to declare "moving"

%% ---- Load Data ----
[file, path] = uigetfile('*.mat', 'Select LAB4 dead band data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp  = load(fullfile(path, file));
vars = fieldnames(tmp);

enc = []; vel = []; vt = []; flag = []; t = [];

for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v, 'Simulink.SimulationData.Dataset')
        try
            enc  = double(v.getElement('encoder_rad').Values.Data(:));
            vel  = double(v.getElement('Kv_velo').Values.Data(:));
            vt   = double(v.getElement('vout').Values.Data(:));
            flag = double(v.getElement('active flag').Values.Data(:));
            t    = double(v.getElement('encoder_rad').Values.Time(:));
        catch e
            error('Could not extract signals: %s\nCheck signal names in Simulink.', e.message);
        end
        break;
    end
end

if isempty(enc)
    error('No Simulink.SimulationData.Dataset found in file.');
end

N = length(enc);
if isempty(t); t = (0:N-1)' * dt; end
fprintf('Loaded %d samples (%.1f s)\n', N, t(end));

%% ---- Trim to active ramp window ----
% Use active flag to isolate only the ramp period
active_idx = find(flag > 0.5);
if isempty(active_idx)
    warning('active flag never went high — using full dataset.');
    active_idx = (1:N)';
end
i_start = active_idx(1);
i_end   = active_idx(end);

enc_r  = enc(i_start:i_end);
vel_r  = vel(i_start:i_end);
vt_r   = vt(i_start:i_end);
t_r    = t(i_start:i_end) - t(i_start);   % t=0 at ramp start
Nr     = length(enc_r);

%% ---- Smooth velocity for reliable threshold crossing ----
vel_sm = movmean(abs(vel_r), SMOOTH_WIN);

%% ---- Position delta signal for detector 2 ----
% |enc[k] - enc[k - ENC_WIN]| over a sliding window
enc_delta = zeros(Nr, 1);
for k = (ENC_WIN+1):Nr
    enc_delta(k) = abs(enc_r(k) - enc_r(k - ENC_WIN));
end
enc_delta_sm = movmean(enc_delta, SMOOTH_WIN);

%% ---- Find ramp peak (split up/down) ----
[~, i_peak] = max(vt_r);

vel_up   = vel_sm(1:i_peak);       vel_dn   = vel_sm(i_peak:end);
enc_up   = enc_delta_sm(1:i_peak); enc_dn   = enc_delta_sm(i_peak:end);
vt_up    = vt_r(1:i_peak);         vt_dn    = vt_r(i_peak:end);
t_up     = t_r(1:i_peak);          t_dn     = t_r(i_peak:end);

%% ---- Detector 1: Kalman velocity threshold ----
i1_move = find(vel_up >= VELO_THR, 1, 'first');
if isempty(i1_move)
    warning('D1: Motor never moved during ramp up. Lower VELO_THR (%.4f rad/s).', VELO_THR);
    V1_start = NaN;  t1_move = NaN;
else
    V1_start = vt_up(i1_move);  t1_move = t_up(i1_move);
end

i1_stop = find(vel_dn >= VELO_THR, 1, 'last');
if isempty(i1_stop)
    warning('D1: Could not detect stop on ramp down.');
    V1_stop = NaN;  t1_stop = NaN;
else
    V1_stop = vt_dn(i1_stop);  t1_stop = t_dn(i1_stop);
end

%% ---- Detector 2: Encoder position change threshold ----
i2_move = find(enc_up >= ENC_THR, 1, 'first');
if isempty(i2_move)
    warning('D2: Encoder never moved during ramp up. Lower ENC_THR (%.5f rad).', ENC_THR);
    V2_start = NaN;  t2_move = NaN;
else
    V2_start = vt_up(i2_move);  t2_move = t_up(i2_move);
end

i2_stop = find(enc_dn >= ENC_THR, 1, 'last');
if isempty(i2_stop)
    warning('D2: Could not detect stop on ramp down.');
    V2_stop = NaN;  t2_stop = NaN;
else
    V2_stop = vt_dn(i2_stop);  t2_stop = t_dn(i2_stop);
end

%% ---- Print results ----
fprintf('\n============================================================\n');
fprintf('  DEAD BAND ANALYSIS RESULTS\n');
fprintf('============================================================\n');
fprintf('  DETECTOR 1 — Kalman velocity  (threshold %.4f rad/s)\n', VELO_THR);
fprintf('    V_start (start move) : %7.4f V   (t = %.2f s)\n', V1_start, t1_move);
fprintf('    V_stop  (stop  move) : %7.4f V   (t = %.2f s)\n', V1_stop,  t1_stop);
fprintf('    Dead band (avg)      : %7.4f V\n', mean([V1_start, V1_stop], 'omitnan'));
fprintf('    Hysteresis           : %7.4f V\n', abs(V1_start - V1_stop));
fprintf('------------------------------------------------------------\n');
fprintf('  DETECTOR 2 — Encoder position (threshold %.5f rad)\n', ENC_THR);
fprintf('    V_start (start move) : %7.4f V   (t = %.2f s)\n', V2_start, t2_move);
fprintf('    V_stop  (stop  move) : %7.4f V   (t = %.2f s)\n', V2_stop,  t2_stop);
fprintf('    Dead band (avg)      : %7.4f V\n', mean([V2_start, V2_stop], 'omitnan'));
fprintf('    Hysteresis           : %7.4f V\n', abs(V2_start - V2_stop));
fprintf('============================================================\n\n');

%% ---- Figure 1: Overview — Detector 1 (velocity) ----
figure('Name','LAB4 Dead Band — Detector 1 (Velocity)','NumberTitle','off',...
       'Units','normalized','Position',[0.02 0.52 0.58 0.42]);

yyaxis left
plot(t_r, vt_r, 'b', 'LineWidth', 1.5, 'DisplayName', 'Voltage [V]');
ylabel('Voltage [V]'); ylim([-0.5, 10]);

yyaxis right
plot(t_r, vel_sm, 'r', 'LineWidth', 1.2, 'DisplayName', '|Kv\_velo| smoothed');
ylabel('|Velocity| [rad/s]');
hold on;
yline(VELO_THR, 'k--', sprintf('Thr %.3f rad/s', VELO_THR), ...
      'LineWidth', 0.8, 'HandleVisibility','off');

if ~isnan(t1_move)
    xline(t1_move, 'g-', sprintf('V_{start}=%.3fV', V1_start), ...
          'LineWidth', 1.8, 'LabelVerticalAlignment','bottom');
end
if ~isnan(t1_stop)
    xline(t1_stop, 'm-', sprintf('V_{stop}=%.3fV',  V1_stop), ...
          'LineWidth', 1.8, 'LabelVerticalAlignment','bottom');
end

grid on; xlabel('Time [s]');
title(sprintf('Detector 1 (Velocity):  V_{start}=%.3fV   V_{stop}=%.3fV   Hyst=%.3fV', ...
      V1_start, V1_stop, abs(V1_start-V1_stop)));
legend('Location','best');

%% ---- Figure 2: Overview — Detector 2 (encoder position) ----
figure('Name','LAB4 Dead Band — Detector 2 (Encoder)','NumberTitle','off',...
       'Units','normalized','Position',[0.02 0.05 0.58 0.42]);

yyaxis left
plot(t_r, vt_r, 'b', 'LineWidth', 1.5, 'DisplayName', 'Voltage [V]');
ylabel('Voltage [V]'); ylim([-0.5, 10]);

yyaxis right
plot(t_r, enc_delta_sm, 'Color',[0.85 0.33 0.10], 'LineWidth', 1.2, ...
     'DisplayName', 'Enc delta smoothed');
ylabel('Position delta [rad]');
hold on;
yline(ENC_THR, 'k--', sprintf('Thr %.4f rad', ENC_THR), ...
      'LineWidth', 0.8, 'HandleVisibility','off');

if ~isnan(t2_move)
    xline(t2_move, 'g-', sprintf('V_{start}=%.3fV', V2_start), ...
          'LineWidth', 1.8, 'LabelVerticalAlignment','bottom');
end
if ~isnan(t2_stop)
    xline(t2_stop, 'm-', sprintf('V_{stop}=%.3fV',  V2_stop), ...
          'LineWidth', 1.8, 'LabelVerticalAlignment','bottom');
end

grid on; xlabel('Time [s]');
title(sprintf('Detector 2 (Encoder):  V_{start}=%.3fV   V_{stop}=%.3fV   Hyst=%.3fV', ...
      V2_start, V2_stop, abs(V2_start-V2_stop)));
legend('Location','best');

%% ---- Figure 3: Hysteresis loops — velocity vs voltage ----
figure('Name','LAB4 Dead Band — Hysteresis Loops','NumberTitle','off',...
       'Units','normalized','Position',[0.62 0.52 0.36 0.88]);

subplot(2,1,1);
plot(vt_up, vel_up, 'b',  'LineWidth', 1.5, 'DisplayName','Ramp up');   hold on;
plot(vt_dn, vel_dn, 'r',  'LineWidth', 1.5, 'DisplayName','Ramp down');
yline(VELO_THR, 'k--', 'Threshold', 'LineWidth', 0.8, 'HandleVisibility','off');
if ~isnan(V1_start); xline(V1_start, 'g-', sprintf('%.3fV', V1_start), 'LineWidth',1.5); end
if ~isnan(V1_stop);  xline(V1_stop,  'm-', sprintf('%.3fV', V1_stop),  'LineWidth',1.5); end
grid on; xlabel('Voltage [V]'); ylabel('|Velocity| [rad/s]');
title('D1: Velocity Hysteresis Loop'); legend('Location','best');

subplot(2,1,2);
plot(vt_up, enc_up, 'b',  'LineWidth', 1.5, 'DisplayName','Ramp up');   hold on;
plot(vt_dn, enc_dn, 'r',  'LineWidth', 1.5, 'DisplayName','Ramp down');
yline(ENC_THR, 'k--', 'Threshold', 'LineWidth', 0.8, 'HandleVisibility','off');
if ~isnan(V2_start); xline(V2_start, 'g-', sprintf('%.3fV', V2_start), 'LineWidth',1.5); end
if ~isnan(V2_stop);  xline(V2_stop,  'm-', sprintf('%.3fV', V2_stop),  'LineWidth',1.5); end
grid on; xlabel('Voltage [V]'); ylabel('Position delta [rad]');
title('D2: Encoder Hysteresis Loop'); legend('Location','best');

sgtitle('Hysteresis Loops  (blue=up, red=down, green=start, magenta=stop)', ...
        'FontWeight','bold');

%% ---- Figure 4: Position & both detectors marked ----
figure('Name','LAB4 Dead Band — Time Traces','NumberTitle','off',...
       'Units','normalized','Position',[0.62 0.05 0.36 0.42]);

subplot(2,1,1);
plot(t_r, enc_r, 'b', 'LineWidth', 1.2);
% D1
if ~isnan(t1_move); xline(t1_move, 'g-',  'D1 start', 'LineWidth',1.5); end
if ~isnan(t1_stop);  xline(t1_stop, 'm-', 'D1 stop',  'LineWidth',1.5); end
% D2
if ~isnan(t2_move); xline(t2_move, 'g--', 'D2 start', 'LineWidth',1.2); end
if ~isnan(t2_stop);  xline(t2_stop, 'm--','D2 stop',  'LineWidth',1.2); end
grid on; ylabel('Position [rad]');
title('Encoder Position  (solid=D1, dashed=D2)');
xlabel('Time [s]');

subplot(2,1,2);
plot(t_r, vel_r,  'Color',[0.75 0.75 0.75], 'LineWidth',0.8, 'DisplayName','Raw vel');
hold on;
plot(t_r, vel_sm .* sign(vel_r + 1e-9), 'r', 'LineWidth',1.2, 'DisplayName','Smoothed vel');
yline( VELO_THR,'k--','LineWidth',0.8,'HandleVisibility','off');
yline(-VELO_THR,'k--','LineWidth',0.8,'HandleVisibility','off');
if ~isnan(t1_move); xline(t1_move, 'g-',  'LineWidth',1.5,'HandleVisibility','off'); end
if ~isnan(t1_stop);  xline(t1_stop, 'm-', 'LineWidth',1.5,'HandleVisibility','off'); end
if ~isnan(t2_move); xline(t2_move, 'g--', 'LineWidth',1.2,'HandleVisibility','off'); end
if ~isnan(t2_stop);  xline(t2_stop, 'm--','LineWidth',1.2,'HandleVisibility','off'); end
grid on; ylabel('Velocity [rad/s]'); xlabel('Time [s]');
title('Kalman Velocity'); legend('Location','best');
