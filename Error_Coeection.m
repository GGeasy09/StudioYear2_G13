clear;
fixed_step = 0.001;

load('Lab1 Part2 Data\Lab1 Part2 Data New\lab1_chirp_pi14_time10_3.mat')
load('Lab1 Part2 Parameter\chirmp_t10_freqpi_4.mat')

raw_data = speed;



% ---- Find when signals start to rise ----
dval_raw = diff(raw_data);
dval_Signal = diff(Sample_Signal);

idx_raw = find(dval_raw > 0, 1, 'first');
idx_signal = find(dval_Signal > 0, 1, 'first');

raise_raw = time(idx_raw);
raise_signal = time(idx_signal);

diffsignal = raise_raw - raise_signal + fixed_step;

% ---- Shift time axis ----
time_shift = (time - diffsignal + fixed_step)+ diffsignal;

% ---- Delete data before t = 0 (your earlier request) ----
idx = time_shift >= 0;      % keep only time ≥ 0
time_shift = time_shift(idx);
raw_data_shift = raw_data(idx); 
time_end = time_shift(end);
% ---- Plot ----
figure;
hold on;
% plot(time_shift, raw_data_shift);   % shifted & trimmed data
plot(time, raw_data);               % original raw data
plot(time_shift, raw_data_shift)
plot(Sample_time, Sample_Signal);   % reference signal

save('Lab1 Part2 Data\Lab1 Part2 Data Solve Error\lab1_chirp_pi14_time10_3.mat', ...
    "raw_data","raw_data_shift","time_shift","time");
