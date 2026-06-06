% ==========================================================================
%  LAB1_Velocity_Analysis.m
%  Compare Kalman velocity estimate (X[1]) against backward finite-difference
%  of encoder position.
%
%  Derivative:  v_fd(k) = ( pos(k) - pos(k - round(DT_WINDOW/Ts)) ) / DT_WINDOW
% ==========================================================================
clear; clc; close all;

%% ---- Settings ----
DT_WINDOW  = 0.10;    % finite-difference window [s]  (larger = smoother)
Ts         = 0.0005;  % sample period [s]  (2 kHz)
T_PRE      = 0.5;     % s before motion to show
T_POST     = 0.5;     % s after motion to show

%% ---- Load data ----
[file, path] = uigetfile('*.mat', 'Select LAB1 data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp  = load(fullfile(path, file));
vars = fieldnames(tmp);

enc_rad = []; kv_velo = []; t = [];

for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v, 'Simulink.SimulationData.Dataset')
        try
            enc_rad = double(v.getElement('encoder_rad').Values.Data(:));
            kv_velo = double(v.getElement('Kv_velo').Values.Data(:));
            t       = double(v.getElement('encoder_rad').Values.Time(:));
        catch e
            error('Signal extraction failed: %s', e.message);
        end
        break;
    end
end

if isempty(enc_rad); error('No dataset found.'); end
if isempty(t); t = (0:length(enc_rad)-1)' * Ts; end

N = length(enc_rad);
fprintf('Loaded %d samples (%.2f s)\n', N, t(end));

%% ---- Backward finite-difference velocity ----
win = round(DT_WINDOW / Ts);   % window in samples
v_fd = zeros(N, 1);
for k = win+1 : N
    v_fd(k) = (enc_rad(k) - enc_rad(k - win)) / DT_WINDOW;
end
% First 'win' samples: simple 1-step difference
for k = 2 : win
    v_fd(k) = (enc_rad(k) - enc_rad(k-1)) / Ts;
end

%% ---- Error ----
err = kv_velo - v_fd;

% Ignore first window (derivative not valid there)
valid = win+1 : N;

rms_err  = rms(err(valid));
mae_err  = mean(abs(err(valid)));
max_err  = max(abs(err(valid)));
r_corr   = corrcoef(kv_velo(valid), v_fd(valid));
corr_val = r_corr(1, 2);
peak_kv  = max(abs(kv_velo));

%% ---- Print metrics ----
fprintf('\n========================================\n');
fprintf('  LAB1 KALMAN VELOCITY ACCURACY\n');
fprintf('  FD window  : %.4f s  (%d samples)\n', DT_WINDOW, win);
fprintf('========================================\n');
fprintf('  RMS  error : %.6f rad/s\n', rms_err);
fprintf('  MAE  error : %.6f rad/s\n', mae_err);
fprintf('  Max  error : %.6f rad/s\n', max_err);
fprintf('  Correlation: %.6f\n',       corr_val);
fprintf('  Peak |Kv|  : %.4f rad/s\n', peak_kv);
fprintf('  Rel. RMS   : %.3f %%  (RMS err / peak Kv)\n', rms_err/peak_kv*100);
fprintf('========================================\n\n');

%% ---- Figure 1: Position ----
figure('Name','LAB1 — Encoder Position','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.55 0.48 0.38]);

plot(t, enc_rad, 'Color',[0 0.55 0], 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Position [rad]');
title('Encoder Position'); grid on;

%% ---- Figure 2: Velocity Comparison ----
figure('Name','LAB1 — Velocity Comparison','NumberTitle','off',...
    'Units','normalized','Position',[0.51 0.55 0.48 0.38]);

subplot(2,1,1);
plot(t, kv_velo, 'b',  'LineWidth', 1.3, 'DisplayName', sprintf('Kalman X[1]')); hold on;
plot(t, v_fd,    '--', 'Color',[0.9 0.6 0], 'LineWidth', 1.0, 'DisplayName', sprintf('FD (dt=%.3fs)', DT_WINDOW));
yline(0, 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
legend('Location','best'); grid on;
ylabel('Velocity [rad/s]');
title(sprintf('Velocity Comparison   |r| = %.4f   Peak = %.3f rad/s', corr_val, peak_kv));

subplot(2,1,2);
plot(t, err, 'r', 'LineWidth', 1.0);
yline(0, 'k-', 'LineWidth', 0.5);
yline( rms_err, 'b--', sprintf('+RMS=%.4f', rms_err), 'LineWidth', 0.8, 'LabelHorizontalAlignment','left');
yline(-rms_err, 'b--', sprintf('-RMS=%.4f', rms_err), 'LineWidth', 0.8, 'LabelHorizontalAlignment','left');
xlabel('Time [s]'); ylabel('Error [rad/s]');
title(sprintf('Kalman − FD  |  RMS=%.4f  MAE=%.4f  Max=%.4f rad/s', rms_err, mae_err, max_err));
grid on;

%% ---- Figure 3: Scatter ----
figure('Name','LAB1 — Scatter','NumberTitle','off',...
    'Units','normalized','Position',[0.26 0.08 0.48 0.40]);

scatter(v_fd(valid), kv_velo(valid), 2, err(valid), 'filled'); hold on;
cb = colorbar; cb.Label.String = 'Error [rad/s]';
colormap(gca, 'jet');
vmin = min([v_fd(valid); kv_velo(valid)]);
vmax = max([v_fd(valid); kv_velo(valid)]);
plot([vmin vmax], [vmin vmax], 'k--', 'LineWidth', 1.2, 'DisplayName', 'y = x (ideal)');
xlabel('FD velocity [rad/s]'); ylabel('Kalman velocity [rad/s]');
title(sprintf('Scatter: Kalman vs FD   r = %.4f', corr_val));
legend('Location','southeast'); grid on; axis equal;
