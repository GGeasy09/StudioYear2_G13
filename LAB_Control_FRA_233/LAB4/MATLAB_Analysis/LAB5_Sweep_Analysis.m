%% LAB5_Sweep_Analysis.m  (fixed for long 1-hour recordings)
% Load ONE long Simulink HDF5 recording that covers the full LAB5 auto-sweep
% (5°→360° in 5° steps, LAB5_REPEATS=5 per target, return-to-home between).
% Segments every forward move, computes overshoot & error-at-hold-end,
% groups by target, plots mean±std per target.
%
% -------------------------------------------------------------------------
% FIXES vs previous version (verified against the actual 817 MB file):
%   1. GLITCH CLEANING — long recordings contain a few dozen corrupted
%      samples per stream (garbage like 1e+189 from interrupted buffer
%      flushes). These broke unique()-based detection. Now cleaned with
%      previous-valid-sample fill.
%   2. SIGNAL DETECTION —
%        * traj_state: tolerance-based two-level test (not exact unique).
%        * positions:  only degree-scaled streams (max > 50°); the file
%          also contains radian copies (max = 2π) that used to confuse it.
%        * ref vs enc: the reversal-count heuristic picked them BACKWARDS.
%          Now: ref = stream whose value at trajectory-complete edges lands
%          exactly on the 2.5° target grid (residual 0.001° vs 0.53°).
%   3. SEGMENTATION — traj_state: 0 = MOVING, 1 = HOLDING.
%      Old code measured the rise→fall window (= the hold, where encoder
%      already sits at target → dist≈0 → EVERY move was skipped).
%      Now: move = fall→rise; overshoot over [move start .. hold+0.5 s];
%      error at 0.5 s into hold.
%   4. Figure 4 indexing bug (results index ≠ edge index) fixed by storing
%      segment indices in results.
%   5. Removed full-length unique() diagnostics (slow on 8.2 M samples).
%
% Verified on LAB5_sweep.mat: 349 forward moves, 72 targets, 4–5 repeats.
%
% ADDED: MAX SPEED & ACCELERATION detection.
%   * Auto-detects the measured OUTPUT-shaft velocity stream (integral test
%     rejects motor-side gear-ratio copies and the reference velocity).
%   * Acceleration derived from velocity (25 ms smoothing).
%   * Max |v| and |a| counted ONLY inside trajectory ranges (state = 0,
%     move start → move complete), over ALL moves incl. return-to-home.
%   * Per-target maxima in Figure 5 + summary table; global max printed.
% -------------------------------------------------------------------------

clear; close all; clc;

% =========================================================================
%  USER SETTINGS
% =========================================================================
MAT_FILE        = '../Robot_Test_Result/LAB5_sweep.mat';   % <-- edit as needed

FS              = 2000;        % Simulink sample rate [Hz]
LAB5_STEP_DEG   = 5.0;         % degrees per target step
LAB5_REPEATS    = 5;           % repeats per target
LAB5_N_TARGETS  = 72;          % 5° .. 360°

SETTLE_TOL_DEG  = 0.1;         % ±° for error-at-hold pass/fail
OVERSHOOT_LIMIT = 1.0;         % % of distance
HOLD_S          = 0.5;         % hold window [s] → error measured at end

% Minimum target to count as a "forward" move (not a return-to-home)
MIN_FWD_DEG     = 1.0;

GLITCH_THRESH   = 1e4;         % |sample| above this = corrupted byte glitch

% =========================================================================
%  LOAD STREAMS  (v7.3 HDF5, packed uint8 bytes)
% =========================================================================
fprintf('Loading %s …\n', MAT_FILE);

use_typecast = false;
try
    di = h5info(MAT_FILE, '/#sigstream#/0/#data#');
    use_typecast = strcmp(di.Datatype.Class, 'H5T_INTEGER');
catch
end
fprintf('HDF5 storage format: %s\n', ...
    ternary(use_typecast, 'packed uint8 bytes (typecast)', 'native double'));

streams = cell(13, 1);
for k = 0:12
    try
        raw = h5read(MAT_FILE, sprintf('/#sigstream#/%d/#data#', k));
        n   = double(h5read(MAT_FILE, sprintf('/#sigstream#/%d/#length#', k)));

        if use_typecast
            if ~isvector(raw)
                if size(raw,1) ~= 8 && size(raw,2) == 8
                    raw = raw';            % [N x 8] → [8 x N]
                end
            end
            d = typecast(uint8(raw(:)), 'double');
            n_samp = n;                    % #length# = sample count
        else
            d = double(raw(:));
            n_samp = n;
        end
        d = d(1 : min(n_samp, numel(d)));

        % --- GLITCH CLEANING (fix #1) -----------------------------------
        bad = ~isfinite(d) | abs(d) > GLITCH_THRESH;
        if any(bad)
            d(bad) = NaN;
            if isnan(d(1)), d(1) = 0; end
            d = fillmissing(d, 'previous');
        end
        streams{k+1} = d;
    catch
        streams{k+1} = [];
    end
end

% Cheap diagnostics (downsampled — fix #5)
fprintf('Stream diagnostics (downsampled):\n');
for k = 0:12
    d = streams{k+1};
    if isempty(d)
        fprintf('  s%d: empty\n', k);
    else
        ds = d(1:200:end);
        fprintf('  s%d: len=%-9d min=%10.4g  max=%10.4g  mean=%10.4g  nuniq~%d\n', ...
                k, numel(d), min(d), max(d), mean(d), numel(unique(round(ds,6))));
    end
end

% =========================================================================
%  DETECT SIGNALS  (fix #2)
% =========================================================================
[ts_idx, ref_idx, enc_idx] = detect_signals(streams, LAB5_STEP_DEG);
fprintf('\nDetected → Trajectory_state: s%d  |  Ref: s%d  |  Enc: s%d\n', ...
        ts_idx, ref_idx, enc_idx);

traj_state = streams{ts_idx + 1};
ref_deg    = streams{ref_idx + 1};
enc_deg    = streams{enc_idx + 1};
N          = numel(enc_deg);

% --- Velocity stream (measured, e.g. Kalman estimate in rad/s) -----------
[vel_idx, vel_scale] = detect_velocity(streams, [ts_idx ref_idx enc_idx], ...
                                       enc_deg, traj_state, FS);
if isnan(vel_idx)
    fprintf('No velocity stream found — deriving velocity from encoder.\n');
    vel_deg_s = movmean([0; diff(enc_deg)] * FS, 151);
else
    fprintf('Detected → Velocity: s%d (scale ×%.4f → deg/s)\n', vel_idx, vel_scale);
    vel_deg_s = streams{vel_idx + 1} * vel_scale;
end

% Acceleration = smoothed derivative of velocity (25 ms moving averages)
ACC_W      = round(0.025 * FS);
acc_deg_s2 = movmean([0; diff(movmean(vel_deg_s, 2*ACC_W+1))] * FS, 2*ACC_W+1);

clear streams raw d;            % free ~700 MB

% =========================================================================
%  EDGES — traj_state: 0 = trajectory RUNNING, 1 = done/holding
%    fall (1→0) = move starts
%    rise (0→1) = move complete → hold window begins
% =========================================================================
b          = traj_state > 0.5;
rise_edges = find(diff(b) > 0) + 1;
fall_edges = find(diff(b) < 0) + 1;

if isempty(rise_edges) || isempty(fall_edges)
    error('No trajectory edges found. Check signal detection.');
end
fprintf('Edges: %d move-starts (falls), %d move-completes (rises)\n', ...
        numel(fall_edges), numel(rise_edges));

% =========================================================================
%  SEGMENT MOVES  (fix #3: one move per fall→rise pair)
% =========================================================================
results = struct('target_deg', {}, 'overshoot_pct', {}, 'err_at_hold', {}, ...
                 'pass', {}, 'i_start', {}, 'i_end', {}, ...
                 'max_speed', {}, 'max_accel', {});

% Global max speed / accel — counted ONLY inside trajectory ranges
% (state = 0, fall→rise), over ALL moves incl. return-to-home.
glob_max_v = 0;  glob_max_v_i = 1;
glob_max_a = 0;  glob_max_a_i = 1;

for m = 1:numel(fall_edges)
    f_i = fall_edges(m);                          % move start
    r_i = rise_edges(find(rise_edges > f_i, 1));  % move complete
    if isempty(r_i), break; end

    % --- Max speed / accel inside this trajectory range ------------------
    [v_pk, vi] = max(abs(vel_deg_s(f_i : r_i)));
    if r_i - f_i > 4*ACC_W   % accel: skip ACC_W edge samples (filter edges)
        [a_pk, ai] = max(abs(acc_deg_s2(f_i+ACC_W : r_i-ACC_W)));
    else
        a_pk = 0; ai = 1;
    end
    if v_pk > glob_max_v, glob_max_v = v_pk; glob_max_v_i = f_i + vi - 1;     end
    if a_pk > glob_max_a, glob_max_a = a_pk; glob_max_a_i = f_i + ACC_W + ai - 1; end

    target = ref_deg(r_i);                        % ref is exact at completion

    % Skip return-to-home moves
    if abs(target) <= MIN_FWD_DEG, continue; end

    dist = abs(target - enc_deg(f_i));            % from move START (fix #3)
    if dist < 0.5, continue; end

    % Overshoot over move + hold window
    hold_end_i = min(N, r_i + round(HOLD_S * FS));
    seg = enc_deg(f_i : hold_end_i);
    if target > 0, peak = max(seg); else, peak = min(seg); end
    overshoot_deg = max(0, (peak - target) * sign(target));
    overshoot_pct = 100 * overshoot_deg / dist;

    % Error at end of hold window (median of last 25 ms vs single noisy sample)
    w0 = max(f_i, hold_end_i - round(0.025 * FS));
    err_at_hold = median(enc_deg(w0 : hold_end_i)) - target;

    pass_all = (overshoot_pct <= OVERSHOOT_LIMIT) && ...
               (abs(err_at_hold) <= SETTLE_TOL_DEG);

    r.target_deg    = target;
    r.overshoot_pct = overshoot_pct;
    r.err_at_hold   = err_at_hold;
    r.pass          = pass_all;
    r.i_start       = f_i;                        % stored for Fig 4 (fix #4)
    r.i_end         = hold_end_i;
    r.max_speed     = v_pk;                       % deg/s, trajectory range only
    r.max_accel     = a_pk;                       % deg/s², trajectory range only
    results(end+1)  = r; %#ok<AGROW>
end

fprintf('Segmented %d forward moves.\n', numel(results));
if isempty(results)
    error('No forward moves found. Check MAT_FILE and signal detection.');
end

% =========================================================================
%  GROUP BY TARGET
% =========================================================================
targets   = LAB5_STEP_DEG : LAB5_STEP_DEG : LAB5_N_TARGETS * LAB5_STEP_DEG;
n_tgt     = numel(targets);

mean_os   = nan(1, n_tgt);  std_os  = nan(1, n_tgt);
mean_err  = nan(1, n_tgt);  std_err = nan(1, n_tgt);
pass_rate = nan(1, n_tgt);
max_v_tgt = nan(1, n_tgt);  max_a_tgt = nan(1, n_tgt);

all_tgt = [results.target_deg];
for ti = 1:n_tgt
    mask = abs(all_tgt - targets(ti)) <= LAB5_STEP_DEG/2;
    if ~any(mask), continue; end
    grp_os   = [results(mask).overshoot_pct];
    grp_err  = [results(mask).err_at_hold];
    grp_pass = [results(mask).pass];
    mean_os(ti)   = mean(grp_os);   std_os(ti)  = std(grp_os);
    mean_err(ti)  = mean(grp_err);  std_err(ti) = std(grp_err);
    pass_rate(ti) = 100 * mean(double(grp_pass));
    max_v_tgt(ti) = max([results(mask).max_speed]);
    max_a_tgt(ti) = max([results(mask).max_accel]);
end

% =========================================================================
%  PLOTS
% =========================================================================

%% Figure 1 — Overshoot vs Target
figure('Name','LAB5 Overshoot vs Target','NumberTitle','off');
errorbar(targets, mean_os, std_os, 'b-o','LineWidth',1.2,'MarkerSize',4);
hold on;
yline(OVERSHOOT_LIMIT,'r--','LineWidth',1.5,'Label','Limit 1%');
xlabel('Target [°]'); ylabel('Overshoot [%]');
title('LAB5 Auto-Sweep — Overshoot per Target (mean ± std)');
grid on;

%% Figure 2 — Error at Hold End vs Target
figure('Name','LAB5 Error @ Hold End','NumberTitle','off');
errorbar(targets, mean_err, std_err, 'g-o','LineWidth',1.2,'MarkerSize',4);
hold on;
yline( SETTLE_TOL_DEG,'r--','LineWidth',1.5,'Label','+0.1°');
yline(-SETTLE_TOL_DEG,'r--','LineWidth',1.5,'Label','-0.1°');
xlabel('Target [°]'); ylabel(sprintf('Error at %.1f s hold end [°]', HOLD_S));
title('LAB5 Auto-Sweep — Steady-State Error per Target (mean ± std)');
grid on;

%% Figure 3 — Pass Rate vs Target
figure('Name','LAB5 Pass Rate','NumberTitle','off');
bar(targets, pass_rate, 'FaceColor',[0.2 0.6 0.2]);
xlabel('Target [°]'); ylabel('Pass rate [%]');
ylim([0 110]);
yline(100,'r--','LineWidth',1.5);
title('LAB5 Auto-Sweep — Pass Rate per Target');
grid on;

%% Figure 4 — All encoder traces (forward moves only)  (fix #4)
figure('Name','LAB5 All Forward Traces','NumberTitle','off');
hold on; colors = lines(n_tgt);
for m = 1:numel(results)
    ti = max(1, min(round(results(m).target_deg / LAB5_STEP_DEG), n_tgt));
    i0 = results(m).i_start;  i1 = results(m).i_end;
    seg_t = (0 : i1 - i0) / FS;
    plot(seg_t, enc_deg(i0:i1), 'Color', [colors(ti,:) 0.3], 'LineWidth', 0.8);
end
xlabel('Time from move start [s]'); ylabel('Encoder [°]');
title('LAB5 — All Forward Move Traces');
grid on;

%% Figure 5 — Max Speed & Accel per Target (trajectory ranges only) [rad units]
figure('Name','LAB5 Max Speed & Accel','NumberTitle','off');
subplot(2,1,1);
plot(targets, deg2rad(max_v_tgt), 'b-o','LineWidth',1.2,'MarkerSize',4);
hold on;
yline(deg2rad(glob_max_v),'r--','LineWidth',1.2, ...
      'Label',sprintf('Global max %.3f rad/s', deg2rad(glob_max_v)));
xlabel('Target [°]'); ylabel('Max |speed| [rad/s]');
title('LAB5 — Max Speed per Target (forward moves, trajectory range only)');
grid on;
subplot(2,1,2);
plot(targets, deg2rad(max_a_tgt), 'm-o','LineWidth',1.2,'MarkerSize',4);
hold on;
yline(deg2rad(glob_max_a),'r--','LineWidth',1.2, ...
      'Label',sprintf('Global max %.2f rad/s²', deg2rad(glob_max_a)));
xlabel('Target [°]'); ylabel('Max |accel| [rad/s²]');
title('LAB5 — Max Acceleration per Target (forward moves, trajectory range only)');
grid on;

%% Summary table
fprintf('\n%-10s  %-10s  %-12s  %-12s  %-10s  %-12s  %-12s\n', ...
        'Target°','N_moves','Mean OS %','Mean Err °','Pass %','Max v rad/s','Max a rad/s²');
fprintf('%s\n', repmat('-',1,86));
for ti = 1:n_tgt
    if isnan(mean_os(ti)), continue; end
    mask = abs(all_tgt - targets(ti)) <= LAB5_STEP_DEG/2;
    fprintf('%-10.1f  %-10d  %-12.3f  %-12.3f  %-10.1f  %-12.3f  %-12.2f\n', ...
            targets(ti), sum(mask), mean_os(ti), mean_err(ti), pass_rate(ti), ...
            deg2rad(max_v_tgt(ti)), deg2rad(max_a_tgt(ti)));
end

total_pass = 100 * mean([results.pass]);
fprintf('\nOverall pass rate: %.1f %%\n', total_pass);

fprintf('\n=== MAX DYNAMICS (all moves incl. returns, trajectory ranges only) ===\n');
fprintf('Max speed : %8.3f rad/s   (%.1f °/s)   at t = %.1f s\n', ...
        deg2rad(glob_max_v), glob_max_v, glob_max_v_i / FS);
fprintf('Max accel : %8.2f rad/s²  (%.0f °/s²)  at t = %.1f s\n', ...
        deg2rad(glob_max_a), glob_max_a, glob_max_a_i / FS);

% =========================================================================
%  HELPER — auto-detect signal streams (fix #2)
% =========================================================================
function [ts_idx, ref_idx, enc_idx] = detect_signals(streams, step_deg)

ts_idx = NaN;  ref_idx = NaN;  enc_idx = NaN;

% 1) Trajectory_state: ≥99.9% of samples within 1e-6 of {0,1}, both present.
%    (Tolerance-based — exact unique() broke on glitch-cleaned data.)
for k = 0:12
    d = streams{k+1};
    if isempty(d), continue; end
    near0 = abs(d)     < 1e-6;
    near1 = abs(d - 1) < 1e-6;
    if mean(near0 | near1) > 0.999 && any(near0) && any(near1) && ...
       mean(d) > 0.05 && mean(d) < 0.99
        ts_idx = k;
        break;
    end
end
if isnan(ts_idx)
    error('Cannot detect Trajectory_state stream. Check stream diagnostics above.');
end

% Trajectory-complete edges (needed to tell ref from enc)
rises = find(diff(streams{ts_idx+1} > 0.5) > 0) + 1;
if isempty(rises)
    error('Trajectory_state has no rising edges.');
end

% 2) Position candidates: degree-scaled only (max > 50° — the file also
%    holds radian copies with max = 2π which must be excluded).
cands = [];
for k = 0:12
    if k == ts_idx, continue; end
    d = streams{k+1};
    if isempty(d), continue; end
    if abs(d(1)) < 2.0 && max(d) > 50
        cands(end+1) = k; %#ok<AGROW>
    end
end
if numel(cands) < 2
    error('Cannot find two degree-scaled position streams (need max > 50°).');
end

% 3) Ref vs Enc: at trajectory-complete edges the REFERENCE sits exactly on
%    the target grid (multiples of step_deg, incl. 0 for home). The encoder
%    has noise/offset. Smallest mean residual to the half-step grid = ref.
grid_step = step_deg / 2;
resid = inf(1, numel(cands));
for c = 1:numel(cands)
    v = streams{cands(c)+1}(rises);
    resid(c) = mean(abs(v - round(v / grid_step) * grid_step));
end
[~, order] = sort(resid);
ref_idx = cands(order(1));
enc_idx = cands(order(2));
fprintf('ref/enc residuals to %.2f° grid: ', grid_step);
fprintf('s%d=%.4f° ', [cands; resid]);
fprintf('\n');

end

% =========================================================================
%  HELPER — auto-detect measured OUTPUT-shaft velocity stream
%  The file contains several velocity-like streams: motor-side copies
%  (× gear ratio), reference velocity, and the measured output velocity.
%  Discriminators (verified on the actual recording):
%    1. corr with d(enc)/dt > 0.7                → tracks the motion
%    2. NOT exactly 0 during holds               → rejects reference velocity
%    3. INTEGRAL TEST: ∫v·dt over the longest move must equal the actual
%       encoder displacement in a clean unit (rad or deg) → rejects
%       motor-side streams (their integral is gear-ratio× too large)
%    4. among passers: largest hold-window std   → measured, not held value
% =========================================================================
function [vel_idx, scale] = detect_velocity(streams, used_idx, enc_deg, traj_state, FS)

vel_idx = NaN;  scale = NaN;

b     = traj_state > 0.5;
rises = find(diff(b) > 0) + 1;
falls = find(diff(b) < 0) + 1;
hold_mask = b;

% Longest move (largest |Δenc| over a fall→rise trajectory range)
D_best = 0; FA = NaN; RI = NaN;
for m = 1:numel(falls)
    fa = falls(m);
    ri = rises(find(rises > fa, 1));
    if isempty(ri), break; end
    if abs(enc_deg(ri) - enc_deg(fa)) > D_best
        D_best = abs(enc_deg(ri) - enc_deg(fa));
        FA = fa;  RI = ri;
    end
end
if isnan(FA), return; end
dD = enc_deg(RI) - enc_deg(FA);                   % actual displacement [deg]

venc_rad = deg2rad(movmean([0; diff(enc_deg)] * FS, 151));
ds = 50;
x  = venc_rad(1:ds:end);

best_holdstd = -inf;
for k = 0:12
    if any(k == used_idx), continue; end
    d = streams{k+1};
    if isempty(d) || numel(d) ~= numel(enc_deg), continue; end

    c = corrcoef(x, d(1:ds:end));
    if abs(c(1,2)) < 0.7, continue; end           % (1) must track motion

    if mean(abs(d(hold_mask)) < 1e-9) > 0.5, continue; end   % (2) ref vel

    I = sum(d(FA:RI)) / FS;                       % (3) integral test
    if I == 0, continue; end
    sc   = dD / I;                                % empirical units→deg/s
    uerr = min(abs(log(abs(sc) / (180/pi))), abs(log(abs(sc))));
    if uerr > 0.5, continue; end                  % not a clean rad/deg unit

    hstd = std(d(hold_mask));                     % (4) prefer measured
    if hstd > best_holdstd
        best_holdstd = hstd;
        vel_idx = k;
        % snap empirical scale to the exact unit constant
        if abs(log(abs(sc) / (180/pi))) < abs(log(abs(sc)))
            scale = sign(sc) * 180/pi;            % stream is rad/s
        else
            scale = sign(sc);                     % stream is deg/s
        end
    end
end

end

function s = ternary(cond, a, b)
if cond; s = a; else; s = b; end
end
