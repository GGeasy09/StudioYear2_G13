% ==========================================================================
%  LAB1_Interactive.m  —  Single Kalman run, edit params and press F5
%
%  Requires encoder_rad and vout_data already in workspace.
%  Run LAB1_Kalman_Analysis.m first to load them, then use this file
%  to quickly test different Q values.
% ==========================================================================

%% ---- EDIT THESE PARAMETERS THEN PRESS F5 ----
Qw   = 5.33e-11;    % velocity process noise    Q[1][1]
Qd   = 5.00e-12;   % disturbance process noise Q[2][2]
R_meas = 3e-8;     % measurement noise

T_WIN = [5, 25];  % plot window [s]
% --------------------------------------------------

%% ---- Check workspace data exists ----
if ~exist('encoder_rad','var') || ~exist('vout_data','var')
    error('Run LAB1_Kalman_Analysis.m first to load encoder_rad and vout_data.');
end

%% ---- Motor & matrices (must match STM32) ----
J=8.6701e-1; B=0.4308; kt=2.2461; km=2.2461; R=0.4821323156; L=0.0002893301219;
dt = 0.0005;

F = [1,     dt,          0,      0;
     0, 1-B/J*dt,    -dt/J, kt/J*dt;
     0,      0,           1,      0;
     0, -km/L*dt,         0, 1-R/L*dt];
G = [0;0;0;dt/L];
H = [1,0,0,0];
Q = diag([0, Qw, Qd, 0]);

%% ---- Run both Kalman variants ----
N = length(encoder_rad);
t = (0:N-1)' * dt;

pos_out   = zeros(N,1); vel_out   = zeros(N,1); dist_out   = zeros(N,1);
pos_ni    = zeros(N,1); vel_ni    = zeros(N,1); dist_ni    = zeros(N,1);

X  = zeros(4,1); P  = diag([0,1,1,1]);   % with input
Xn = zeros(4,1); Pn = diag([0,1,1,1]);   % no input

for k = 1:N
    z = encoder_rad(k);
    u = vout_data(k);

    % --- Normal Kalman (with input) ---
    X  = F*X  + G*u;  P  = F*P*F'  + Q;
    S  = H*P*H'  + R_meas; K  = (P*H')/S;
    IKH = eye(4)-K*H;
    X  = X  + K*(z - H*X);   P  = IKH*P*IKH'  + K*R_meas*K';
    pos_out(k) = X(1); vel_out(k) = X(2); dist_out(k) = X(3);

    % --- No-input Kalman (u = 0) ---
    Xn = F*Xn;         Pn = F*Pn*F' + Q;
    Sn = H*Pn*H' + R_meas; Kn = (Pn*H')/Sn;
    IKHn = eye(4)-Kn*H;
    Xn = Xn + Kn*(z - H*Xn); Pn = IKHn*Pn*IKHn' + Kn*R_meas*Kn';
    pos_ni(k) = Xn(1); vel_ni(k) = Xn(2); dist_ni(k) = Xn(3);
end

err_out = abs(encoder_rad - pos_out) * 57.2958;
err_ni  = abs(encoder_rad - pos_ni)  * 57.2958;

%% ---- Velocity from position (backward finite difference, 0.1 s window) ----
FD_WIN  = 0.10;   % seconds
fd_samp = round(FD_WIN / dt);
v_fd = zeros(N,1);
for k = fd_samp+1 : N
    v_fd(k) = (encoder_rad(k) - encoder_rad(k - fd_samp)) / FD_WIN;
end

rms_fd_out = rms(vel_out(fd_samp+1:end) - v_fd(fd_samp+1:end));
rms_fd_ni  = rms(vel_ni (fd_samp+1:end) - v_fd(fd_samp+1:end));

%% ---- Mean error (despiked, within window) ----
mask = (t >= T_WIN(1)) & (t <= T_WIN(2));

calc_mean = @(e) mean(e(e <= median(e) + 3*mad(e,1)));
mean_err    = calc_mean(err_out(mask));
mean_err_ni = calc_mean(err_ni(mask));

%% ---- Plot ----
figure(99); clf;
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% Tracking Error
nexttile;
plot(t, err_out, 'b',  'LineWidth', 1.2, 'DisplayName','With input'); hold on;
plot(t, err_ni,  '--', 'Color',[0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName','No input');
yline(mean_err,    '-b',  sprintf('%.4f°', mean_err),    'LineWidth',1, 'LabelHorizontalAlignment','left');
yline(mean_err_ni, '--',  sprintf('%.4f°', mean_err_ni), 'LineWidth',1, 'LabelHorizontalAlignment','right', 'Color',[0.85 0.33 0.10]);
xlim(T_WIN); grid on; ylabel('Error [deg]'); legend('Location','best');
title(sprintf('Tracking Error  |  With input: %.4f°   No input: %.4f°', mean_err, mean_err_ni));

% Velocity
nexttile;
plot(t, vel_out, 'b',  'LineWidth', 1.2, 'DisplayName','With input'); hold on;
plot(t, vel_ni,  '--', 'Color',[0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName','No input');
plot(t, v_fd,    'k:', 'LineWidth', 1.2, 'DisplayName','FD (0.1 s)');
xlim(T_WIN); grid on; ylabel('Velocity [rad/s]'); legend('Location','best');
title(sprintf('Velocity Estimate  |  RMS err vs FD:  with-input=%.4f   no-input=%.4f  rad/s', rms_fd_out, rms_fd_ni));

% Disturbance
nexttile;
plot(t, dist_out, 'b',  'LineWidth', 1.2, 'DisplayName','With input'); hold on;
plot(t, dist_ni,  '--', 'Color',[0.85 0.33 0.10], 'LineWidth', 1.2, 'DisplayName','No input');
xlim(T_WIN); grid on; ylabel('\tau_{d} [Nm]'); legend('Location','best');
title('Disturbance Estimate');
xlabel('Time [s]');

sgtitle(sprintf('Qw=%.2e   Qd=%.2e   R=%.2e', Qw, Qd, R_meas), ...
        'FontWeight','bold','FontSize',10);
