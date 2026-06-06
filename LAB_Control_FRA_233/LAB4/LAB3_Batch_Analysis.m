% ==========================================================================
%  LAB3_Batch_Analysis.m
%  Load all .mat files in LAB3_Result/, compute cascade metrics for each,
%  plot all curves together and produce a comparison table.
%
%  Metrics per experiment:
%   RMS / Max position error   [deg]  (during trajectory)
%   RMS / Max velocity error   [rad/s]
%   Overshoot                  [% of travel]
%   Settling time              [s]  (±settle_band deg from target)
%   Final SS error             [deg]
%   Control effort RMS & peak  [V]
%   FF contribution ratio      [%]
% ==========================================================================
clear; clc; close all;

%% ---- Settings ----
RESULT_DIR  = fullfile(fileparts(mfilename('fullpath')), 'LAB3_Result');
dt          = 0.0005;
settle_band = 1.0;    % deg  ±band for settling criterion
T_POST      = 2.0;    % s after trajectory end to include in plots

%% ---- Find files ----
files = dir(fullfile(RESULT_DIR, '*.mat'));
if isempty(files)
    error('No .mat files found in %s', RESULT_DIR);
end
n = numel(files);
fprintf('Found %d file(s) in LAB3_Result\n\n', n);

%% ---- Preallocate ----
labels        = cell(n,1);
rms_pos_errs  = nan(n,1);
max_pos_errs  = nan(n,1);
rms_vel_errs  = nan(n,1);
max_vel_errs  = nan(n,1);
overshoots    = nan(n,1);
settle_times  = nan(n,1);
ss_errors     = nan(n,1);
rms_ctrls     = nan(n,1);
peak_ctrls    = nan(n,1);
ff_ratios     = nan(n,1);

colors = lines(n);

%% ---- Figure setup ----
fig_pos = figure('Name','LAB3 Batch — Position Error','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.53 0.48 0.42]);
hold on; grid on;
xlabel('Time from traj start [s]'); ylabel('Position error [deg]');
title('Position Tracking Error — All Experiments');

fig_vel = figure('Name','LAB3 Batch — Velocity Error','NumberTitle','off',...
    'Units','normalized','Position',[0.51 0.53 0.48 0.42]);
hold on; grid on;
xlabel('Time from traj start [s]'); ylabel('Velocity error [rad/s]');
title('Velocity Tracking Error — All Experiments');

fig_ctrl = figure('Name','LAB3 Batch — Control Effort','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.05 0.48 0.42]);
hold on; grid on;
xlabel('Time from traj start [s]'); ylabel('Voltage [V]');
title('Control Effort — All Experiments');

fig_pos_track = figure('Name','LAB3 Batch — Position Tracking','NumberTitle','off',...
    'Units','normalized','Position',[0.51 0.05 0.48 0.42]);
hold on; grid on;
xlabel('Time from traj start [s]'); ylabel('Position [deg]');
title('Position Tracking — All Experiments');

%% ---- Process each file ----
for fi = 1:n
    fname = files(fi).name;
    label = strrep(fname(1:end-4), '_', ' ');
    labels{fi} = label;
    fpath = fullfile(RESULT_DIR, fname);

    tmp  = load(fpath);
    vars = fieldnames(tmp);

    enc_deg=[]; kv_pos=[]; kv_velo=[]; traj_pos=[]; traj_velo=[];
    ctrl=[]; ref_ff=[]; dist_ff=[]; traj_complete=[]; traj_deg=[]; t=[];

    for vi = 1:numel(vars)
        v = tmp.(vars{vi});
        if isa(v,'Simulink.SimulationData.Dataset')
            try
                enc_deg       = double(v.getElement('encoder_pos_degree').Values.Data(:));
                kv_pos        = double(v.getElement('Kv_pos').Values.Data(:));
                kv_velo       = double(v.getElement('Kv_velo').Values.Data(:));
                traj_pos      = double(v.getElement('traj_pos').Values.Data(:));
                traj_velo     = double(v.getElement('traj_velo').Values.Data(:));
                ctrl          = double(v.getElement('control effort').Values.Data(:));
                ref_ff        = double(v.getElement('reference_ff').Values.Data(:));
                dist_ff       = double(v.getElement('disturbance_ff').Values.Data(:));
                traj_complete = double(v.getElement('Trajectory_state').Values.Data(:));
                traj_deg      = double(v.getElement('traj_pos_degree').Values.Data(:));
                t             = double(v.getElement('encoder_pos_degree').Values.Time(:));
            catch e
                warning('File %s: signal extraction failed — %s', fname, e.message);
            end
            break;
        end
    end

    if isempty(enc_deg)
        warning('File %s: no data, skipping.', fname); continue;
    end

    N = length(enc_deg);
    if isempty(t); t = (0:N-1)'*dt; end

    %% detect trajectory window
    traj_d   = diff(traj_complete);
    falling  = find(traj_d < -0.5);
    rising   = find(traj_d >  0.5);

    if isempty(falling)
        warning('File %s: trajectory never ran, skipping.', fname); continue;
    end
    i_start = falling(end) + 1;
    rising_after = rising(rising > i_start);
    if isempty(rising_after)
        i_end = N;
    else
        i_end = rising_after(1) + 1;
    end

    target_deg     = traj_deg(i_end);
    traj_amplitude = abs(target_deg - traj_deg(i_start));
    if traj_amplitude < 0.1
        warning('File %s: tiny trajectory amplitude, skipping.', fname); continue;
    end

    i_win_end  = min(N, i_end + round(T_POST/dt));
    i_traj     = i_start:i_end;
    i_settle   = i_end:i_win_end;
    t_rel      = t - t(i_start);

    %% errors
    pos_err_deg = traj_deg - enc_deg;
    vel_err     = traj_velo - kv_velo;

    rms_pos_errs(fi) = rms(pos_err_deg(i_traj));
    max_pos_errs(fi) = max(abs(pos_err_deg(i_traj)));
    rms_vel_errs(fi) = rms(vel_err(i_traj));
    max_vel_errs(fi) = max(abs(vel_err(i_traj)));

    %% overshoot [%]
    enc_post = enc_deg(i_settle);
    if target_deg > traj_deg(i_start)
        os_deg = max(0, max(enc_post) - target_deg);
    else
        os_deg = max(0, target_deg - min(enc_post));
    end
    overshoots(fi) = os_deg / traj_amplitude * 100;

    %% SS error & settling
    ss_s      = max(1, round(0.8*numel(i_settle)));
    final_val = mean(enc_deg(i_settle(ss_s:end)));
    ss_errors(fi) = target_deg - final_val;

    band      = settle_band;
    unsettled = find(abs(enc_deg(i_settle) - target_deg) > band);
    if isempty(unsettled)
        settle_times(fi) = 0;
    else
        settle_times(fi) = (unsettled(end)-1) * dt;
    end

    %% control effort
    rms_ctrls(fi)  = rms(ctrl(i_traj));
    peak_ctrls(fi) = max(abs(ctrl(i_traj)));
    pid_comp       = ctrl - ref_ff - dist_ff;
    ff_ratios(fi)  = (rms(ref_ff(i_traj)) + rms(dist_ff(i_traj))) / (rms_ctrls(fi) + 1e-9) * 100;

    %% print
    fprintf('[%s]\n', label);
    fprintf('  RMS pos err=%.4f°  Max=%.4f°  RMS vel err=%.4f  OS=%.3f%%  SS=%.4f°  St=%.3fs  RMS V=%.3f  FF=%.1f%%\n\n', ...
        rms_pos_errs(fi), max_pos_errs(fi), rms_vel_errs(fi), ...
        overshoots(fi), ss_errors(fi), settle_times(fi), rms_ctrls(fi), ff_ratios(fi));

    %% plot window
    win = i_start:i_win_end;
    tw  = t_rel(win);

    figure(fig_pos);
    plot(tw, pos_err_deg(win), 'Color',colors(fi,:), 'LineWidth',1.2, 'DisplayName',label);
    xline(t_rel(i_end), '--', 'Color',colors(fi,:), 'LineWidth',0.8, 'HandleVisibility','off');

    figure(fig_vel);
    plot(tw, vel_err(win), 'Color',colors(fi,:), 'LineWidth',1.2, 'DisplayName',label);
    xline(t_rel(i_end), '--', 'Color',colors(fi,:), 'LineWidth',0.8, 'HandleVisibility','off');

    figure(fig_ctrl);
    plot(tw, ctrl(win), 'Color',colors(fi,:), 'LineWidth',1.2, 'DisplayName',label);
    xline(t_rel(i_end), '--', 'Color',colors(fi,:), 'LineWidth',0.8, 'HandleVisibility','off');

    figure(fig_pos_track);
    plot(tw, enc_deg(win),  'Color',colors(fi,:), 'LineWidth',1.2, 'DisplayName',[label ' actual']);
    plot(tw, traj_deg(win), '--', 'Color',colors(fi,:)*0.6, 'LineWidth',0.8, 'HandleVisibility','off');
    xline(t_rel(i_end), '--', 'Color',colors(fi,:), 'LineWidth',0.8, 'HandleVisibility','off');
end

figure(fig_pos);    yline(0,'k--','LineWidth',0.8,'HandleVisibility','off'); legend('Location','best');
figure(fig_vel);    yline(0,'k--','LineWidth',0.8,'HandleVisibility','off'); legend('Location','best');
figure(fig_ctrl);   yline(0,'k--','LineWidth',0.8,'HandleVisibility','off'); legend('Location','best');
figure(fig_pos_track); legend('Location','best');

%% ---- Console table ----
valid = ~isnan(rms_pos_errs);
fprintf('\n%s\n', repmat('=',1,100));
fprintf('%-22s  %9s  %9s  %9s  %9s  %9s  %9s  %9s  %7s\n', ...
    'Experiment','RMSpos°','Maxpos°','RMSvel','OS%','SSe°','Settle s','RMS V','FF%');
fprintf('%s\n', repmat('-',1,100));
for fi = 1:n
    if ~valid(fi); continue; end
    fprintf('%-22s  %9.4f  %9.4f  %9.4f  %9.3f  %9.4f  %9.4f  %9.4f  %7.1f\n', ...
        labels{fi}, rms_pos_errs(fi), max_pos_errs(fi), rms_vel_errs(fi), ...
        overshoots(fi), ss_errors(fi), settle_times(fi), rms_ctrls(fi), ff_ratios(fi));
end
fprintf('%s\n', repmat('=',1,100));

%% ---- Bar chart comparison ----
xl = labels(valid);
nv = sum(valid);
metrics_mat = [rms_pos_errs(valid), overshoots(valid), settle_times(valid), ss_errors(valid), rms_ctrls(valid)];
metric_names = {'RMS Pos Err [°]','Overshoot [%]','Settle Time [s]','SS Error [°]','RMS Ctrl [V]'};

figure('Name','LAB3 Batch — Metric Comparison','NumberTitle','off',...
    'Units','normalized','Position',[0.01 0.00 0.98 0.48]);

for mi = 1:5
    subplot(1,5,mi);
    b = bar(metrics_mat(:,mi), 'FaceColor','flat');
    b.CData = colors(valid,:);
    set(gca,'XTickLabel',xl,'XTick',1:nv,'XTickLabelRotation',30,'FontSize',8);
    ylabel(metric_names{mi}); title(metric_names{mi}); grid on;
    for k = 1:nv
        text(k, metrics_mat(k,mi)*1.02, sprintf('%.3f', metrics_mat(k,mi)), ...
            'HorizontalAlignment','center','FontSize',7);
    end
end
sgtitle('LAB3 Cascade Control — Parameter Comparison','FontWeight','bold');
