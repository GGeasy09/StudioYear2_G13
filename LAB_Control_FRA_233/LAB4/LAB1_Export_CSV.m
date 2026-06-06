% ==========================================================================
%  LAB1_Export_CSV.m
%  Export LAB1 Simulink log to CSV for LAB1_Velocity_Analysis.html
%
%  Output columns:  time, encoder_rad, Kv_velo
%  (Kv_velo = Kalman velocity estimate X[1])
% ==========================================================================
clear; clc;

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
if isempty(t); t = (0:length(enc_rad)-1)' * 0.0005; end

out_file = fullfile(path, strrep(file, '.mat', '_velocity.csv'));
T = table(t, enc_rad, kv_velo, 'VariableNames', {'time','encoder_rad','Kv_velo'});
writetable(T, out_file);

fprintf('Exported %d samples → %s\n', length(t), out_file);
fprintf('Open LAB1_Velocity_Analysis.html and drag this CSV onto the drop zone.\n');
