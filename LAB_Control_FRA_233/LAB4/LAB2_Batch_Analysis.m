% ==========================================================================
%  LAB2_Batch_Analysis.m
%  Load all .mat files in LAB2_Result folder, analyse each step response,
%  plot all curves together and produce a comparison table.
%
%  File naming convention (used as legend label):
%    e.g.  Kp1_Ki10.mat   Kp2_Ki5.mat   etc.
% ==========================================================================
clear; clc; close all;

%% ---- Settings ---- (edit here) ----
RESULT_DIR   = fullfile(fileparts(mfilename('fullpath')), 'LAB2_Result');
dt           = 0.0005;   % 2 kHz
settle_band  = 0.05;     % ±5% settling criterion
T_POST       = 3.0;      % seconds to show AFTER the step

%% ---- Find all .mat files ----
files = dir(fullfile(RESULT_DIR, '*.mat'));
if isempty(files)
    error('No .mat files found in %s', RESULT_DIR);
end
n_files = numel(files);
fprintf('Found %d file(s) in LAB2_Result\n\n', n_files);

%% ---- Preallocate results table ----
labels      = cell(n_files, 1);
rise_times  = nan(n_files, 1);
settle_times= nan(n_files, 1);
overshoots  = nan(n_files, 1);
ss_errors   = nan(n_files, 1);
setpoints   = nan(n_files, 1);

colors = lines(n_files);

%% ---- Figure setup ----
fig1 = figure('Name','LAB2 - All Velocity Responses','NumberTitle','off',...
               'Units','normalized','Position',[0.02 0.52 0.60 0.42]);
hold on; grid on;
xlabel('Time [s]'); ylabel('Velocity [rad/s]');
title('Inner Loop Velocity Step Response — All Experiments');

fig2 = figure('Name','LAB2 - All Tracking Errors','NumberTitle','off',...
               'Units','normalized','Position',[0.63 0.52 0.35 0.42]);
hold on; grid on; yline(0,'k--','LineWidth',0.8);
xlabel('Time [s]'); ylabel('Error [rad/s]');
title('Velocity Tracking Error (setpoint − actual)');

%% ---- Process each file ----
for fi = 1:n_files
    fname  = files(fi).name;
    label  = strrep(fname(1:end-4), '_', ' ');   % strip .mat, replace _ for legend
    labels{fi} = label;
    fpath  = fullfile(RESULT_DIR, fname);

    % --- Load ---
    tmp  = load(fpath);
    vars = fieldnames(tmp);
    vel_actual = []; vel_cmd = []; t = [];

    for vi = 1:numel(vars)
        v = tmp.(vars{vi});
        if isa(v, 'Simulink.SimulationData.Dataset')
            try
                vel_actual = double(v.getElement('Kv_velo').Values.Data(:));
                vel_cmd    = double(v.getElement('Velo_setpoint').Values.Data(:));
                t          = double(v.getElement('Kv_velo').Values.Time(:));
            catch
                warning('File %s: could not extract signals, skipping.', fname);
            end
            break;
        end
    end

    if isempty(vel_actual)
        warning('File %s: no data found, skipping.', fname);
        continue;
    end

    N = length(vel_actual);
    if isempty(t); t = (0:N-1)' * dt; end

    % --- Detect single step: find where setpoint crosses 50% of its range ---
    sp_max  = max(vel_cmd);
    sp_min  = min(vel_cmd);
    if (sp_max - sp_min) < 0.1
        warning('File %s: setpoint barely changes, skipping.', fname);
        continue;
    end
    at_max   = vel_cmd >= sp_max * 0.99;
    edges    = find(diff([0; at_max]) == 1);   % all rising edges into final setpoint
    i0       = edges(end);                     % use the LAST step

    if isempty(edges)
        warning('File %s: no step to final setpoint detected, skipping.', fname);
        continue;
    end
    fprintf('  [%s] %d step(s) found → using last at t=%.2fs\n', label, numel(edges), (i0-1)*dt);

    % Extract segment from step moment, t=0 at step
    i_step = i0;
    i_end  = min(N, i_step + round(T_POST/dt));

    sp  = sp_max;
    v0  = sp_min;
    amp = sp - v0;
    setpoints(fi) = sp;

    seg_post = vel_actual(i_step:i_end);
    t_post   = (0:length(seg_post)-1)' * dt;
    n_seg    = length(seg_post);

    % --- Metrics on post-step segment ---
    t_post = (0:n_seg-1)' * dt;
    t10 = find(seg_post >= v0 + 0.10*amp, 1);
    t90 = find(seg_post >= v0 + 0.90*amp, 1);
    if ~isempty(t10) && ~isempty(t90)
        rise_times(fi) = t_post(t90) - t_post(t10);
    end

    overshoots(fi) = max(0, (max(seg_post) - sp) / amp * 100);

    ss_start  = max(1, round(0.8*n_seg));
    final_val = mean(seg_post(ss_start:end));
    ss_errors(fi) = sp - final_val;

    band      = settle_band * amp;
    unsettled = find(abs(seg_post - final_val) > band);
    settle_times(fi) = isempty(unsettled)*0 + ~isempty(unsettled)*t_post(unsettled(end));

    % --- Plot: t=0 at step moment ---
    figure(fig1);
    plot(t_post, seg_post, 'Color', colors(fi,:), 'LineWidth', 1.3, 'DisplayName', label);

    % --- Plot tracking error (post-step) ---
    figure(fig2);
    err = sp - seg_post;
    plot(t_post, err, 'Color', colors(fi,:), 'LineWidth', 1.2, 'DisplayName', label);

    fprintf('[%s]  SP=%.1f  Rt=%.4fs  St=%.4fs  OS=%.2f%%  SSe=%.4f rad/s\n', ...
            label, sp, rise_times(fi), settle_times(fi), overshoots(fi), ss_errors(fi));
end

figure(fig1);
yline(sp_max, 'k--', 'Setpoint', 'LineWidth', 1.0, 'HandleVisibility','off');
legend('Location','best');
figure(fig2); legend('Location','best');

%% ---- Comparison Table ----
fprintf('\n%s\n', repmat('=',1,75));
fprintf('%-20s  %8s  %10s  %10s  %10s  %10s\n', ...
        'Experiment','SP(r/s)','Rise(s)','Settle(s)','Overshoot%','SSe(r/s)');
fprintf('%s\n', repmat('-',1,75));
for fi = 1:n_files
    fprintf('%-20s  %8.2f  %10.4f  %10.4f  %10.2f  %10.4f\n', ...
            labels{fi}, setpoints(fi), rise_times(fi), ...
            settle_times(fi), overshoots(fi), ss_errors(fi));
end
fprintf('%s\n', repmat('=',1,75));

%% ---- Figure 3: Bar chart comparison ----
figure('Name','LAB2 - Metrics Comparison','NumberTitle','off',...
       'Units','normalized','Position',[0.02 0.05 0.95 0.42]);

valid = ~isnan(rise_times);
xl    = labels(valid);

subplot(1,4,1);
bar(rise_times(valid), 'FaceColor','flat', 'CData', colors(valid,:));
set(gca,'XTickLabel',xl,'XTick',1:sum(valid),'XTickLabelRotation',30);
ylabel('Rise Time [s]'); title('Rise Time'); grid on;

subplot(1,4,2);
bar(settle_times(valid), 'FaceColor','flat', 'CData', colors(valid,:));
set(gca,'XTickLabel',xl,'XTick',1:sum(valid),'XTickLabelRotation',30);
ylabel('Settling Time [s]'); title('Settling Time (±5% final)'); grid on;

subplot(1,4,3);
bar(overshoots(valid), 'FaceColor','flat', 'CData', colors(valid,:));
set(gca,'XTickLabel',xl,'XTick',1:sum(valid),'XTickLabelRotation',30);
ylabel('Overshoot [%]'); title('Overshoot'); grid on;

subplot(1,4,4);
bar(ss_errors(valid), 'FaceColor','flat', 'CData', colors(valid,:));
set(gca,'XTickLabel',xl,'XTick',1:sum(valid),'XTickLabelRotation',30);
ylabel('SS Error [rad/s]'); title('Steady State Error'); grid on;

sgtitle('LAB2 Inner Loop — Parameter Comparison', 'FontWeight','bold');
