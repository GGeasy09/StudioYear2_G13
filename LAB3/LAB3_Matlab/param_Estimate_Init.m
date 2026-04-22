clear;
% --- Default Inputs ---
g = 9.81;
k = 29.415;
b = 0;
m_def = 0.115;
mass_0 = 0.149;
init_pos_def = 0.0978;
offset_time_def = 0.3;

% --- Load Data ---
target_file = 'LAB3_Matlab_Result\attempt_7.mat';
if exist(target_file, 'file')
    load(target_file);
else
    warning('Data file not found. Using script defaults.');
end

% --- Check/Assign Variables ---
if ~exist('m', 'var'), m = m_def; end
if ~exist('Initial_Point', 'var'), Initial_Point = init_pos_def; end
if ~exist('offset_time', 'var'), offset_time = offset_time_def; end




% --- Data Extraction ---
ts = data.getElement("Offset_Distance");
ts2 = data.getElement("Offset_V2");

% --- Process Dataset 1 (Distance) ---
original_t1 = ts.Values.Time;
idx1 = original_t1 >= offset_time_def;

% Cast value to double while indexing
ts_pos = double(ts.Values.Data(idx1)); 
t_new1 = original_t1(idx1) - offset_time_def;

% --- Process Dataset 2 (V2) ---
original_t2 = ts2.Values.Time;
idx2 = original_t2 >= offset_time_def;

% Cast value to double while indexing
ts_pos2 = double(ts2.Values.Data(idx2));
t_new2 = original_t2(idx2) - offset_time_def;

% --- Plotting ---
figure;
subplot(2,1,1);
plot(t_new1, ts_pos, 'b', 'LineWidth', 1.2);
ylabel('Distance (double)');
grid on;

subplot(2,1,2);
plot(t_new2, ts_pos2, 'r', 'LineWidth', 1.2);
ylabel('V2 (double)');
xlabel('Time (s) [Shifted]');
grid on;