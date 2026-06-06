% ==========================================================================
%  LAB2_InnerLoop_Analysis.m
%  Analyse inner-loop velocity step response from recorded STM32 data.
%
%  Expected logged signals (Simulink Log Export):
%   Kv_velo       — Kalman velocity estimate [rad/s]   (TX field 2)
%   velo_setpoint — velocity setpoint [rad/s]           (TX field 5)
%
%  Metrics computed per step:
%   Rise time    — 10% → 90% of step amplitude
%   Settling time — stays within ±5% of final value
%   Overshoot    — peak above setpoint [%]
%   Steady state error — mean error in last 20% of step window
% ==========================================================================
clear; clc; close all;

%% ---- Timing ----
dt = 0.0005;   % 2 kHz

%% ---- Load Data ----
[file, path] = uigetfile('*.mat', 'Select LAB2 logged data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp  = load(fullfile(path, file));
vars = fieldnames(tmp);

vel_actual  = [];
vel_cmd     = [];
t           = [];

for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v, 'Simulink.SimulationData.Dataset')
        vel_actual = double(v.getElement('Kv_velo').Values.Data(:));
        vel_cmd    = double(v.getElement('Velo_setpoint').Values.Data(:));
        t          = double(v.getElement('Kv_velo').Values.Time(:));
        break;
    end
end

if isempty(vel_actual)
    error('Could not find Kv_velo / velo_setpoint. Check signal names in Simulink.');
end

N = length(vel_actual);
if isempty(t); t = (0:N-1)' * dt; end
fprintf('Loaded %d samples (%.1f s)\n', N, t(end));

%% ---- Detect Step Changes in Setpoint ----
d_cmd      = diff(vel_cmd);
step_thr   = 0.1;                        % min step size [rad/s]
step_idx   = find(abs(d_cmd) > step_thr) + 1;

% Keep only rising steps (positive change)
step_idx   = step_idx(vel_cmd(step_idx) > vel_cmd(step_idx-1));

fprintf('Found %d rising step(s)\n', numel(step_idx));

%% ---- Compute Response Metrics ----
settle_band = 0.05;   % ±5% settling criterion

metrics = struct('step_time',{},'setpoint',{},'rise_time',{},...
                 'settle_time',{},'overshoot',{},'ss_error',{});

for si = 1:numel(step_idx)
    i0 = step_idx(si);

    % Find end of this step (next step or end of data)
    if si < numel(step_idx)
        i1 = step_idx(si+1) - 1;
    else
        i1 = N;
    end

    sp     = vel_cmd(i0);               % setpoint value
    v0     = vel_cmd(i0 - 1);           % value before step
    amp    = sp - v0;                   % step amplitude
    if amp <= 0; continue; end

    seg    = vel_actual(i0:i1);
    t_seg  = t(i0:i1) - t(i0);
    n_seg  = length(seg);

    % Rise time (10% → 90%)
    t10    = find(seg >= v0 + 0.10*amp, 1);
    t90    = find(seg >= v0 + 0.90*amp, 1);
    rt     = (isempty(t10)||isempty(t90)) * NaN + ...
             (~isempty(t10)&&~isempty(t90)) * (t_seg(t90)-t_seg(t10));

    % Overshoot
    peak   = max(seg);
    os     = max(0, (peak - sp) / amp * 100);

    % Steady state error (last 20% of segment)
    ss_start  = max(1, round(0.8 * n_seg));
    final_val = mean(seg(ss_start:end));   % actual settled value
    ss_err    = sp - final_val;            % positive = below setpoint

    % Settling time: when signal FIRST enters and stays within
    % ±5% of the ACTUAL final value (not setpoint)
    band      = settle_band * amp;
    unsettled = find(abs(seg - final_val) > band);
    if isempty(unsettled)
        st = 0;
    else
        st = t_seg(unsettled(end));
    end

    m.step_time  = t(i0);
    m.setpoint   = sp;
    m.rise_time  = rt;
    m.settle_time= st;
    m.overshoot  = os;
    m.ss_error   = ss_err;
    metrics(end+1) = m;

    fprintf('\nStep %d at t=%.2fs  →  setpoint=%.2f rad/s\n', si, t(i0), sp);
    if isnan(rt)
        fprintf('  Rise time    : N/A (velocity never reached 90%% of setpoint)\n');
    else
        fprintf('  Rise time    : %.4f s\n', rt);
    end
    fprintf('  Settling time: %.4f s  (±%.0f%%)\n', st, settle_band*100);
    fprintf('  Overshoot    : %.2f %%\n', os);
    fprintf('  SS error     : %.4f rad/s\n', ss_err);
end

%% ---- Figure 1: Full Time Response ----
figure('Name','LAB2 - Velocity Response','NumberTitle','off', ...
       'Units','normalized','Position',[0.02 0.55 0.55 0.38]);
plot(t, vel_cmd,    'k--', 'LineWidth', 1.2, 'DisplayName','Setpoint'); hold on;
plot(t, vel_actual, 'b',   'LineWidth', 1.2, 'DisplayName','Kalman vel');
grid on; xlabel('Time [s]'); ylabel('Velocity [rad/s]');
title('Inner Loop Velocity Response'); legend('Location','best');

%% ---- Figure 2: Zoomed per-step subplots ----
if numel(metrics) > 0
    n_steps = numel(metrics);
    figure('Name','LAB2 - Step Response Zoom','NumberTitle','off', ...
           'Units','normalized','Position',[0.02 0.05 0.95 0.42]);

    for si = 1:n_steps
        i0 = step_idx(si);
        if si < numel(step_idx)
            i1 = step_idx(si+1) - 1;
        else
            i1 = N;
        end
        i1 = min(i1, i0 + round(3.0/dt));   % cap at 3s window

        seg   = vel_actual(i0:i1);
        t_seg = t(i0:i1) - t(i0);
        sp    = metrics(si).setpoint;

        subplot(1, n_steps, si);
        ss_s    = max(1, round(0.8*length(seg)));
        fv      = mean(seg(ss_s:end));
        bd      = settle_band * (vel_cmd(i0) - vel_cmd(max(1,i0-1)));

        plot(t_seg, vel_cmd(i0:i1), 'k--', 'LineWidth',1.2); hold on;
        plot(t_seg, seg,            'b',   'LineWidth',1.2);
        yline(sp,        'r:',  'LineWidth',1.0);           % setpoint
        yline(fv,        'm--', 'LineWidth',0.8);           % actual final value
        yline(fv + bd,   'g:',  'LineWidth',0.8);           % ±5% of final value
        yline(fv - bd,   'g:',  'LineWidth',0.8);
        grid on; xlabel('Time [s]'); ylabel('Vel [rad/s]');
        title(sprintf('Step %.1f rad/s\nRt=%.3fs  OS=%.1f%%\nSt=%.3fs  SSe=%.4f', ...
              sp, metrics(si).rise_time, metrics(si).overshoot, ...
              metrics(si).settle_time, metrics(si).ss_error), 'FontSize',8);
    end
    sgtitle('Step Response Details   (green = ±5% band)', 'FontWeight','bold');
end

%% ---- Figure 3: Tracking Error ----
figure('Name','LAB2 - Tracking Error','NumberTitle','off', ...
       'Units','normalized','Position',[0.57 0.55 0.40 0.38]);
plot(t, vel_cmd - vel_actual, 'r', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('Error [rad/s]');
title('Velocity Tracking Error  (setpoint − actual)');
yline(0, 'k--', 'LineWidth', 0.8);
