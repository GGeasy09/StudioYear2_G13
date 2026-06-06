% ==========================================================================
%  LAB1_Kalman_Analysis.m
%  Post-process recorded STM32 data through all 10 Kalman variants.
%
%  Kalman runs at 2 kHz (dt=0.0005s) matching STM32 exactly.
%  Each 400Hz sample drives 5 sub-steps (predict+update) using the
%  same encoder measurement — closest match to STM32 behavior with
%  available data resolution.
%
%  LAB1 TX packet fields (columns of recorded_data):
%   1  Kalman.X[0]         — STM32 normal Kalman pos  [rad]
%   2  Kalman.X[1]         — STM32 normal Kalman vel  [rad/s]
%   3  Kalman.X[2]         — STM32 normal Kalman tau_dist
%   4  Kalman.X[3]         — STM32 normal Kalman i_current
%   5  Kalman_NoInput.X[0] — STM32 no-input Kalman pos
%   6  Kalman_NoInput.X[1] — STM32 no-input Kalman vel
%   7  Kalman_NoInput.X[2] — STM32 no-input Kalman tau_dist
%   8  Kalman_NoInput.X[3] — STM32 no-input Kalman i_current
%   9  encoder.encoder_rad — raw encoder position [rad]
%  10  Kalman.X[1]         — velocity (duplicate, kept for compat)
%  11  vout                — applied voltage [V]
% ==========================================================================
clear; clc; close all;

%% ---- Motor Parameters (must match STM32 main.c / my_motor) ----
J  = 8.6701e-1;
B  = 0.4308;
kt = 2.2461;
km = 2.2461;
R  = 0.4821323156;
L  = 0.0002893301219;

%% ---- Timing ----
dt    = 0.0005;    % 2 kHz — STM32 KALMAN_FREQ = TX rate (no decimation)
dt_tx = 0.0005;    % 2 kHz — TX now sends every tick
N_sub = 1;         % 1:1 match — no sub-steps needed

%% ---- State-Space Matrices (identical to STM32 kalman.c) ----
% States: x = [theta, omega, tau_dist, i_current]
F = [1,      dt,            0,       0;
     0,  1-B/J*dt,      -dt/J,  kt/J*dt;
     0,      0,             1,       0;
     0, -km/L*dt,           0,  1-R/L*dt];

G = [0; 0; 0; dt/L];       % voltage → current state
H = [1, 0, 0, 0];          % only position measured

%% ---- Noise Base Values (STM32 lab_config.h defaults) ----
Qw_base = 5.33e-9;   % process_noise   → velocity state Q[1][1]
Qd_base = 5.00e-10;  % disturbance_noise → tau_dist state Q[2][2]
R_meas  = 3.00e-8;   % measurement noise

%% ---- Q Ratio Sets: [Qw_scale, Qd_scale] (R always = 1) ----
Q_ratios = [
    1,      1;       %  1:1:1
    0.01,   0.1;    %  0.01:0.01:1
    0.1,    0.01;     %  100:100:1
    0.1,   0.1;     %  0.01:100:1
    1,    0.01;    %  100:0.01:1
];
ratio_labels = {
    'Q_\omega:Q_d:R = 1:1:1',
    'Q_\omega:Q_d:R = 0.01:0.01:1',
    'Q_\omega:Q_d:R = 100:100:1',
    'Q_\omega:Q_d:R = 0.01:100:1',
    'Q_\omega:Q_d:R = 100:0.01:1',
};
n_ratios = size(Q_ratios, 1);

%% ---- Load Recorded Data (Simulink Log Export: encoder_rad + vout) ----
[file, path] = uigetfile('*.mat', 'Select LAB1 logged data (.mat)');
if isequal(file, 0); error('No file selected.'); end
tmp = load(fullfile(path, file));

encoder_rad = [];
vout_data   = [];
t           = [];

% Find the Dataset variable
vars = fieldnames(tmp);
for vi = 1:numel(vars)
    v = tmp.(vars{vi});
    if isa(v, 'Simulink.SimulationData.Dataset')
        enc_elem  = v.getElement('encoder_rad');
        vout_elem = v.getElement('vout');
        encoder_rad = double(enc_elem.Values.Data(:));
        vout_data   = double(vout_elem.Values.Data(:));
        t           = double(enc_elem.Values.Time(:));
        break;
    end
end

% Fallback: direct named variables in workspace
if isempty(encoder_rad) && isfield(tmp, 'encoder_rad')
    encoder_rad = double(tmp.encoder_rad.Data(:));
    vout_data   = double(tmp.vout.Data(:));
    t           = double(tmp.encoder_rad.Time(:));
end

if isempty(encoder_rad)
    error('Could not find encoder_rad / vout in the file.');
end

N_samples = length(encoder_rad);
if isempty(t); t = (0:N_samples-1)' * dt_tx; end

fprintf('Loaded %d samples (%.2f s)\n', N_samples, t(end));

%% ---- Run All 10 Kalman Variants ----
% Preallocate: [N_samples x n_ratios] for each signal, 2 modes
pos_normal   = zeros(N_samples, n_ratios);
vel_normal   = zeros(N_samples, n_ratios);
dist_normal  = zeros(N_samples, n_ratios);
err_normal   = zeros(N_samples, n_ratios);

pos_noinput  = zeros(N_samples, n_ratios);
vel_noinput  = zeros(N_samples, n_ratios);
dist_noinput = zeros(N_samples, n_ratios);
err_noinput  = zeros(N_samples, n_ratios);

P0 = diag([0, 1, 1, 1]);   % initial covariance (matches STM32)

for r = 1:n_ratios
    Qw = Q_ratios(r,1) * Qw_base;
    Qd = Q_ratios(r,2) * Qd_base;
    Q  = diag([0, Qw, Qd, 0]);

    for mode = 1:2   % 1 = normal (with voltage input), 2 = no-input
        X = zeros(4,1);
        P = P0;
        pos_out  = zeros(N_samples,1);
        vel_out  = zeros(N_samples,1);
        dist_out = zeros(N_samples,1);

        for k = 1:N_samples
            z = encoder_rad(k);
            u = vout_data(k) * (mode == 1);   % no-input: force u=0

            % 5 sub-steps at 2 kHz per 400 Hz sample
            for s = 1:N_sub
                % Predict
                X = F*X + G*u;
                P = F*P*F' + Q;
                % Update (use same z for all sub-steps)
                S = H*P*H' + R_meas;
                K = (P*H') / S;
                IKH = eye(4) - K*H;
                X = X + K*(z - H*X);
                P = IKH*P*IKH' + K*R_meas*K';   % Joseph form (numerically stable)
            end

            pos_out(k)  = X(1);
            vel_out(k)  = X(2);
            dist_out(k) = X(3);
        end

        err_deg = abs(encoder_rad - pos_out) * 57.2958;   % [deg]

        if mode == 1
            pos_normal(:,r)  = pos_out;
            vel_normal(:,r)  = vel_out;
            dist_normal(:,r) = dist_out;
            err_normal(:,r)  = err_deg;
        else
            pos_noinput(:,r)  = pos_out;
            vel_noinput(:,r)  = vel_out;
            dist_noinput(:,r) = dist_out;
            err_noinput(:,r)  = err_deg;
        end
    end

    fprintf('Done ratio %d/%d: %s\n', r, n_ratios, ratio_labels{r});
end

%% ---- Save Results ----
save('kalman_results.mat', ...
    't', 'encoder_rad', 'vout_data', ...
    'pos_normal',  'vel_normal',  'dist_normal',  'err_normal', ...
    'pos_noinput', 'vel_noinput', 'dist_noinput', 'err_noinput', ...
    'Q_ratios', 'ratio_labels', 'dt', 'dt_tx');
fprintf('Results saved to kalman_results.mat\n');

%% ---- Plot: 3 figures, each with 5 subplots (one per Q-ratio) ----
lw = 1.5;
c  = [0 0.447 0.741];   % single blue — same signal, compare shape across ratios

T_WIN = [5, 25];   % time window [s]

% --- Figure 1: Tracking Error ---
figure('Name','LAB1 - Tracking Error','NumberTitle','off', ...
       'Units','normalized','Position',[0.0 0.05 0.32 0.88]);

mask_win = (t >= T_WIN(1)) & (t <= T_WIN(2));   % time window mask

for r = 1:n_ratios
    subplot(n_ratios, 1, r);

    err = err_noinput(:,r);
    plot(t, err, 'Color', c, 'LineWidth', lw);
    xlim(T_WIN); grid on; ylabel('Error [deg]');

    % Mean tracking error after spike removal (within window)
    err_win   = err(mask_win);
    spike_thr = median(err_win) + 3 * mad(err_win, 1);   % median + 3×MAD threshold
    clean     = err_win(err_win <= spike_thr);
    mean_err  = mean(clean);

    % Draw mean line + annotation
    hold on;
    yline(mean_err, '--r', 'LineWidth', 1.2);
    text(T_WIN(2) - 0.1, mean_err, sprintf('  %.3f°', mean_err), ...
         'Color','r', 'FontSize', 8, 'HorizontalAlignment','right', 'VerticalAlignment','bottom');
    hold off;

    title(sprintf('%s   |   Mean err = %.3f°', ratio_labels{r}, mean_err), 'FontSize', 9);
    if r == n_ratios; xlabel('Time [s]'); end
end
sgtitle('Tracking Error — No-Input Kalman', 'FontWeight','bold');

% --- Figure 2: Velocity ---
figure('Name','LAB1 - Velocity (No-Input)','NumberTitle','off', ...
       'Units','normalized','Position',[0.34 0.05 0.32 0.88]);
for r = 1:n_ratios
    subplot(n_ratios, 1, r);
    plot(t, vel_noinput(:,r), 'Color', c, 'LineWidth', lw);
    xlim(T_WIN); grid on; ylabel('Vel [rad/s]');
    title(ratio_labels{r}, 'FontSize', 9);
    if r == n_ratios; xlabel('Time [s]'); end
end
sgtitle('Velocity Estimate — No-Input Kalman', 'FontWeight','bold');

% --- Figure 3: Disturbance ---
figure('Name','LAB1 - Disturbance (No-Input)','NumberTitle','off', ...
       'Units','normalized','Position',[0.67 0.05 0.32 0.88]);
for r = 1:n_ratios
    subplot(n_ratios, 1, r);
    plot(t, dist_noinput(:,r), 'Color', c, 'LineWidth', lw);
    xlim(T_WIN); grid on; ylabel('\tau_{d} [Nm]');
    title(ratio_labels{r}, 'FontSize', 9);
    if r == n_ratios; xlabel('Time [s]'); end
end
sgtitle('Disturbance Estimate — No-Input Kalman', 'FontWeight','bold');

%% ---- Summary Table: Mean Absolute Error ----
fprintf('\n%-25s  %12s  %12s\n', 'Ratio', 'MAE Normal', 'MAE No-input');
fprintf('%s\n', repmat('-', 1, 55));
for r = 1:n_ratios
    fprintf('%-25s  %10.4f°  %10.4f°\n', ...
        strrep(ratio_labels{r},'Q_\omega:Q_d:R = ',''), ...
        mean(err_normal(:,r)), mean(err_noinput(:,r)));
end
