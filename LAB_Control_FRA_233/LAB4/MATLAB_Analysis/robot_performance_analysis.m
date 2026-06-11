% =========================================================================
%  robot_performance_analysis.m
%
%  Reads every Robot_Test_Result/P2P_*.mat file (Simulink v7.3 HDF5 format),
%  auto-detects signal streams, and computes per-move performance metrics.
%
%  CRITERIA:
%    Overshoot     < 1%   of distance
%    Settling time < 0.5 s  (from trajectory Complete until position stays
%                             within +-max(0.5deg, 2%*dist) for 100 ms)
%
%  SIGNAL AUTO-DETECTION (works even if stream order varies between files):
%    Trajectory_state : only binary 0/1 stream (0=running, 1=done)
%    traj_pos_degree  : starts near 0, monotonically rises to ~target deg
%    encoder_pos_degree: starts near 0, can overshoot target, may have
%                         steady-state error
%
%  USAGE:
%    1. Set DATA_DIR to the folder containing P2P_*.mat files.
%    2. Run -- analysis and plots appear automatically.
% =========================================================================

clear; close all; clc;

%% ---- CONFIGURATION -------------------------------------------------
DATA_DIR         = '../Robot_Test_Result';   % relative to this script
FS               = 2000;         % sample rate [Hz]  (Simulink 2 kHz)

SETTLE_TOL_DEG   = 0.1;          % tolerance band +/-deg (fixed, no percentage)
SETTLE_TOL_PCT   = 0.0;          % disabled — using fixed deg only
SETTLE_CONFIRM_S = 0.10;         % must stay in band for this long [s]
POST_TRAJ_S      = 2.0;          % max window to search after Complete [s]

OVERSHOOT_LIMIT  = 1.0;          % % of distance
SETTLING_LIMIT   = 0.1;          % seconds

%% ---- FIND FILES ----------------------------------------------------
mats = dir(fullfile(DATA_DIR, 'P2P_*.mat'));
if isempty(mats)
    error('No P2P_*.mat files found in %s', DATA_DIR);
end
% Sort by numeric target value
targets_arr = zeros(1, numel(mats));
for i = 1:numel(mats)
    tok = regexp(mats(i).name, 'P2P_(\d+)', 'tokens');
    targets_arr(i) = str2double(tok{1}{1});
end
[targets_arr, si] = sort(targets_arr);
mats = mats(si);
nFiles = numel(mats);
fprintf('Found %d files: %s ... %s\n\n', nFiles, mats(1).name, mats(end).name);

%% ---- PROCESS EACH FILE ---------------------------------------------
CONFIRM_N = round(SETTLE_CONFIRM_S * FS);
POST_N    = round(POST_TRAJ_S * FS);

% Pre-allocate result struct array
res = struct( ...
    'fname',        repmat({''}, 1, nFiles), ...
    'target_deg',   num2cell(zeros(1,nFiles)), ...
    'os_pct',       num2cell(zeros(1,nFiles)), ...
    'settle_s',     num2cell(NaN(1,nFiles)), ...
    'err_at_hold',  num2cell(zeros(1,nFiles)), ...
    'ss_err_deg',   num2cell(zeros(1,nFiles)), ...
    'pass_os',      num2cell(false(1,nFiles)), ...
    'pass_err',     num2cell(false(1,nFiles)), ...
    'pass',         num2cell(false(1,nFiles)), ...
    't',            cell(1,nFiles), ...
    'enc_deg',      cell(1,nFiles), ...
    'ref_deg',      cell(1,nFiles), ...
    'rise_i',       num2cell(zeros(1,nFiles)) ...
);
valid_mask = false(1, nFiles);

for fi = 1:nFiles
    fname      = fullfile(DATA_DIR, mats(fi).name);
    target_deg = targets_arr(fi);

    % --- Load all 13 streams ----------------------------------------
    streams = cell(13, 1);
    for k = 0:12
        try
            raw = h5read(fname, sprintf('/#sigstream#/%d/#data#', k));
            n   = h5read(fname, sprintf('/#sigstream#/%d/#length#', k));
            d   = typecast(uint8(raw(:)), 'double');
            streams{k+1} = d(1:n);
        catch
            streams{k+1} = [];
        end
    end

    % --- Auto-detect signals ----------------------------------------
    [ts_idx, ref_idx, enc_idx] = detect_signals(streams, target_deg);
    if isnan(ts_idx) || isnan(enc_idx)
        fprintf('%-14s  SKIP -- signal detection failed (ts=%d ref=%d enc=%d)\n', ...
            mats(fi).name, ts_idx, ref_idx, enc_idx);
        continue;
    end

    traj_state = streams{ts_idx + 1};
    enc_deg    = streams{enc_idx + 1};
    ref_deg    = streams{ref_idx + 1};
    N          = length(enc_deg);
    t          = (0 : N-1)' / FS;

    % --- Find trajectory-complete rising edge (0->1), skip first 5% --
    edges = find(diff(traj_state > 0.5) > 0) + 1;
    edges = edges(edges > round(0.05 * N));
    if isempty(edges)
        fprintf('%-14s  SKIP -- no Complete edge found\n', mats(fi).name);
        continue;
    end
    rise_i = edges(1);

    % --- Overshoot --------------------------------------------------
    peak   = max(enc_deg);
    os_pct = max(0, (peak - target_deg) / target_deg * 100);

    % --- Settling time (from complete edge) -------------------------
    tol     = max(SETTLE_TOL_DEG, SETTLE_TOL_PCT * target_deg);
    in_band = abs(enc_deg - target_deg) <= tol;
    st      = NaN;
    search_end = min(N - CONFIRM_N, rise_i + POST_N);
    for k = rise_i : search_end
        if all(in_band(k : k + CONFIRM_N - 1))
            st = (k - rise_i) / FS;
            break;
        end
    end

    % --- Error at end of 0.5 s hold window -------------------------
    hold_end_i  = min(N, rise_i + round(0.5 * FS));
    err_at_hold = enc_deg(hold_end_i) - target_deg;   % signed [deg]
    pass_err    = abs(err_at_hold) <= SETTLE_TOL_DEG;

    % --- Steady-state error (mean of last 100 ms) -------------------
    tail_n     = round(0.10 * FS);
    ss_err     = mean(enc_deg(max(1,N-tail_n) : end)) - target_deg;

    % --- Pass/Fail --------------------------------------------------
    pass_os = os_pct <= OVERSHOOT_LIMIT;

    % --- Store ------------------------------------------------------
    res(fi).fname        = mats(fi).name;
    res(fi).target_deg   = target_deg;
    res(fi).os_pct       = os_pct;
    res(fi).settle_s     = st;
    res(fi).err_at_hold  = err_at_hold;
    res(fi).ss_err_deg   = ss_err;
    res(fi).pass_os      = pass_os;
    res(fi).pass_err     = pass_err;
    res(fi).pass         = pass_os && pass_err;
    res(fi).t            = t;
    res(fi).enc_deg      = enc_deg;
    res(fi).ref_deg      = ref_deg;
    res(fi).rise_i       = rise_i;
    valid_mask(fi)       = true;

    % --- Print row --------------------------------------------------
    if pass_os && pass_err,        status = 'PASS         ';
    elseif ~pass_os && ~pass_err,  status = 'FAIL(OS+ERR) ';
    elseif ~pass_os,               status = 'FAIL(OS)     ';
    else,                          status = 'FAIL(ERR)    '; end

    fprintf('%-14s  target=%3.0f deg  peak=%6.3f deg  OS=%6.3f%%  err@0.5s=%+.3f deg  %s\n', ...
        mats(fi).name, target_deg, peak, os_pct, err_at_hold, status);
end

%% ---- COLLECT VALID RESULTS -----------------------------------------
res  = res(valid_mask);
nm   = numel(res);
if nm == 0, error('No valid moves processed.'); end

os_all       = [res.os_pct];
err_hold_all = [res.err_at_hold];
err_all      = [res.ss_err_deg];
tgt_all      = [res.target_deg];
pass_all     = logical([res.pass]);
n_pass       = sum(pass_all);
n_fail       = nm - n_pass;

%% ---- SUMMARY PRINT -------------------------------------------------
fprintf('\n========== PERFORMANCE SUMMARY (%d moves) ==========\n', nm);
fprintf('PASS: %d  FAIL: %d  (%.1f%% pass rate)\n', n_pass, n_fail, 100*n_pass/nm);
fprintf('\nOvershoot [%% of distance]:\n');
fprintf('  max  = %.3f%%  (limit %.1f%%)\n', max(os_all), OVERSHOOT_LIMIT);
fprintf('  mean = %.3f%%\n', mean(os_all));
fprintf('\nError at end of 0.5 s hold (limit +/-%.1f deg):\n', SETTLE_TOL_DEG);
fprintf('  max |err| = %.3f deg\n', max(abs(err_hold_all)));
fprintf('  mean|err| = %.3f deg\n', mean(abs(err_hold_all)));
fprintf('  passed    = %d/%d\n', sum(logical([res.pass_err])), nm);
fprintf('====================================================\n\n');

%% ---- COLOURS -------------------------------------------------------
C_PASS = [0.15 0.65 0.20];
C_FAIL = [0.85 0.15 0.15];
C_WARN = [0.95 0.60 0.10];
dot_col = repmat(C_FAIL, nm, 1);
dot_col(pass_all, :) = repmat(C_PASS, n_pass, 1);

%% ---- FIGURE 1: SUMMARY CHARTS --------------------------------------
figure('Name','Robot Performance Analysis', ...
       'Position',[30 30 1600 900], 'Color','w');

% 1 -- Overshoot vs target angle
subplot(2,3,1);
scatter(tgt_all, os_all, 70, dot_col, 'filled'); hold on;
yline(OVERSHOOT_LIMIT,'r--','LineWidth',1.8,'Label','1% limit');
xlabel('Target [deg]'); ylabel('Overshoot [%]');
title('Overshoot vs Target');
grid on; box on; xlim([0 max(tgt_all)+5]);

% 2 -- Error at 0.5 s hold end vs target
subplot(2,3,2);
bar(tgt_all, abs(err_hold_all), 0.6, 'FaceColor',[0.4 0.6 0.9],'EdgeColor','none');
hold on;
yline(SETTLE_TOL_DEG,'r--','LineWidth',1.8,'Label',sprintf('+/-%.1f deg limit',SETTLE_TOL_DEG));
xlabel('Target [deg]'); ylabel('|Error at 0.5 s hold end| [deg]');
title('Error at End of Hold Window vs Target');
grid on; box on; xlim([0 max(tgt_all)+5]);

% 3 -- Steady-state error vs target
subplot(2,3,3);
bar(tgt_all, abs(err_all), 0.6, 'FaceColor',[0.4 0.6 1.0],'EdgeColor','none');
hold on;
yline(SETTLE_TOL_DEG,'r--','LineWidth',1.5, ...
    'Label',sprintf('%.1f deg tolerance', SETTLE_TOL_DEG));
xlabel('Target [deg]'); ylabel('|Steady-State Error| [deg]');
title('Steady-State Error vs Target');
grid on; box on;

% 4 -- Overshoot histogram
subplot(2,3,4);
histogram(os_all,'BinWidth',0.5,'FaceColor',[0.30 0.60 0.90],'EdgeColor','w');
hold on;
xline(OVERSHOOT_LIMIT,'r--','LineWidth',1.8,'Label','limit');
xlabel('Overshoot [%]'); ylabel('Count');
title('Overshoot Distribution');
grid on; box on;

% 5 -- Error at hold end histogram
subplot(2,3,5);
histogram(abs(err_hold_all),'BinWidth',0.05,'FaceColor',[0.95 0.65 0.20],'EdgeColor','w');
hold on;
xline(SETTLE_TOL_DEG,'r--','LineWidth',1.8,'Label','limit');
xlabel('|Error at 0.5 s hold end| [deg]'); ylabel('Count');
title(sprintf('Hold-End Error Distribution  (%d/%d pass)', sum(logical([res.pass_err])), nm));
grid on; box on;

% 6 -- Pass/Fail pie
ax6 = subplot(2,3,6);
fail_os   = sum(~[res.pass_os] &  [res.pass_err]);
fail_err  = sum( [res.pass_os] & ~[res.pass_err]);
fail_both = sum(~[res.pass_os] & ~[res.pass_err]);
pv   = [n_pass, fail_os, fail_err, fail_both];
pl   = {sprintf('PASS (%d)',n_pass), ...
        sprintf('Fail OS only (%d)',fail_os), ...
        sprintf('Fail Err only (%d)',fail_err), ...
        sprintf('Fail both (%d)',fail_both)};
mask = pv > 0;
pie(pv(mask), pl(mask));
colormap(ax6, [C_PASS; C_FAIL; C_WARN; 0.50 0 0]);
title('Pass/Fail Breakdown');

sgtitle(sprintf('Robot Performance -- %d moves | %d PASS / %d FAIL (%.0f%%)', ...
    nm, n_pass, n_fail, 100*n_pass/nm), 'FontSize',14,'FontWeight','bold');

%% ---- FIGURE 2: POSITION TRACES (all moves) -------------------------
nc = ceil(sqrt(nm));
nr = ceil(nm / nc);
figure('Name','Position Traces','Position',[50 50 1400 900],'Color','w');

for fi = 1:nm
    m      = res(fi);
    rise_i = m.rise_i;
    enc    = m.enc_deg;
    ref    = m.ref_deg;
    t_vec  = m.t;
    tgt    = m.target_deg;

    % Window: 0.5 s before complete to 1.5 s after
    i0  = max(1, rise_i - round(0.5*FS));
    i1  = min(length(enc), rise_i + round(1.5*FS));
    idx = i0:i1;
    t_w = t_vec(idx) - t_vec(rise_i);   % t=0 at trajectory Complete

    ax = subplot(nr, nc, fi);
    plot(t_w, enc(idx), 'Color',[0.1 0.4 0.8], 'LineWidth',1.4); hold on;
    plot(t_w, ref(idx), 'k--', 'LineWidth',0.8);
    yline(tgt, 'r:', 'LineWidth',1.0);
    tol = max(SETTLE_TOL_DEG, SETTLE_TOL_PCT * tgt);
    yline(tgt + tol, 'Color',[0.0 0.6 0.0],'LineStyle',':','LineWidth',0.8);
    yline(tgt - tol, 'Color',[0.0 0.6 0.0],'LineStyle',':','LineWidth',0.8);
    xline(0, 'k:', 'LineWidth',0.8);   % trajectory complete moment
    if ~isnan(m.settle_s)
        xline(m.settle_s,'Color',C_PASS,'LineWidth',1.2);
    end

    % Colour axes by pass/fail
    col = C_PASS; if ~m.pass, col = C_FAIL; end
    ax.XColor = col; ax.YColor = col; ax.LineWidth = 1.3;

    if isnan(m.settle_s), st_s = 'ST=NaN';
    else, st_s = sprintf('ST=%.2fs', m.settle_s); end
    title(sprintf('%ddeg  OS=%.2f%%  %s', round(tgt), m.os_pct, st_s), 'FontSize',8);
    xlabel('t - Complete [s]','FontSize',7);
    ylabel('[deg]','FontSize',7);
    grid on; box on;
    set(ax,'FontSize',7);
end
sgtitle('Position traces  (t=0 = trajectory Complete | green = tolerance band | vertical green = settled)', ...
    'FontSize',11,'FontWeight','bold');

%% ---- SAVE RESULTS --------------------------------------------------
save('robot_performance_results.mat', 'res');
fprintf('Results saved to robot_performance_results.mat\n');

%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

function [ts_idx, ref_idx, enc_idx] = detect_signals(streams, target_deg)
% Auto-detect which stream indices (0-based) hold:
%   ts_idx  = Trajectory_state (binary 0/1)
%   ref_idx = traj_pos_degree  (monotonic reference, ends at target)
%   enc_idx = encoder_pos_degree (actual position, may overshoot / have SS error)
% Returns NaN for any undetected stream.

ts_idx  = NaN;
ref_idx = NaN;
enc_idx = NaN;

% 1) Trajectory_state: unique values = {0,1}, mean in (0.4, 0.99)
for k = 0:12
    d = streams{k+1};
    if isempty(d), continue; end
    u = unique(d);
    if numel(u) == 2 && u(1) == 0 && u(2) == 1 && mean(d) > 0.4 && mean(d) < 0.99
        ts_idx = k;
        break;
    end
end

% 2) Degree-range streams: first sample near 0, max in (0.5x, 1.6x) target
cands = zeros(0, 2);   % [k, n_decrease]
for k = 0:12
    if k == ts_idx, continue; end
    d = streams{k+1};
    if isempty(d), continue; end
    if abs(d(1)) < 0.5 && max(d) > 0.5*target_deg && max(d) < 1.6*target_deg
        n_dec = sum(diff(d) < 0);
        cands(end+1, :) = [k, n_dec]; %#ok<AGROW>
    end
end
if isempty(cands), return; end

% Sort by n_decrease ascending: least noisy = reference trajectory
cands = sortrows(cands, 2);
ref_idx = cands(1, 1);
if size(cands,1) >= 2
    enc_idx = cands(2, 1);
end
end
