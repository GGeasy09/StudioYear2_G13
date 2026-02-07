%% Define All File
%% Define All Files (CORRECT WAY)

basePred = fullfile('C:','Users','ACER','Documents','GitHub',...
    'StudioYear2_G13','Lab1 Part2 Data','Lab1 Part2 Data Predicted Val');

baseVal = fullfile('C:','Users','ACER','Documents','GitHub',...
    'StudioYear2_G13','Lab1 Part2 Data','Lab1 Part2 Data Solve Error');

prediction_val(1) = struct('Signal','step12v_1p',...
    'Filename', fullfile(basePred,'step12v_1.mat'));
prediction_val(2) = struct('Signal','step12v_2p',...
    'Filename', fullfile(basePred,'step12v_2.mat'));
prediction_val(3) = struct('Signal','step12v_3p',...
    'Filename', fullfile(basePred,'step12v_3.mat'));

% Add members 4-6 for 9v (_1,_2,_3)
prediction_val(4) = struct('Signal','step9v_1p',...
    'Filename', fullfile(basePred,'step9v_1.mat'));
prediction_val(5) = struct('Signal','step9v_2p',...
    'Filename', fullfile(basePred,'step9v_2.mat'));
prediction_val(6) = struct('Signal','step9v_3p',...
    'Filename', fullfile(basePred,'step9v_3.mat'));

% Add members 7-9 for 6v (_1,_2,_3)
prediction_val(7) = struct('Signal','step6v_1p',...
    'Filename', fullfile(basePred,'step6v_1.mat'));
prediction_val(8) = struct('Signal','step6v_2p',...
    'Filename', fullfile(basePred,'step6v_2.mat'));
prediction_val(9) = struct('Signal','step6v_3p',...
    'Filename', fullfile(basePred,'step6v_3.mat'));


validation_val(1) = struct('Signal','step12v_1v',...
    'Filename', fullfile(baseVal,'lap1_step_v12_2000hz_1.mat'));
validation_val(2) = struct('Signal','step12v_2v',...
    'Filename', fullfile(baseVal,'lap1_step_v12_2000hz_2.mat'));
validation_val(3) = struct('Signal','step12v_3v',...
    'Filename', fullfile(baseVal,'lap1_step_v12_2000hz_3.mat'));

% Add members 4-6 for 9v (_1,_2,_3)
validation_val(4) = struct('Signal','step9v_1v',...
    'Filename', fullfile(baseVal,'lap1_step_v9_2000hz_1.mat'));
validation_val(5) = struct('Signal','step9v_2v',...
    'Filename', fullfile(baseVal,'lap1_step_v9_2000hz_2.mat'));
validation_val(6) = struct('Signal','step9v_3v',...
    'Filename', fullfile(baseVal,'lap1_step_v9_2000hz_3.mat'));

% Add members 7-9 for 6v (_1,_2,_3)
validation_val(7) = struct('Signal','step6v_1v',...
    'Filename', fullfile(baseVal,'lap1_step_v6_2000hz_1.mat'));
validation_val(8) = struct('Signal','step6v_2v',...
    'Filename', fullfile(baseVal,'lap1_step_v6_2000hz_2.mat'));
validation_val(9) = struct('Signal','step6v_3v',...
    'Filename', fullfile(baseVal,'lap1_step_v6_2000hz_3.mat'));

%% Free Working Space
Np = length(prediction_val);   % number of predictions (3)
Nv = length(validation_val);   % number of validations (3)

RMSE_matrix = zeros(Np, Nv);   % preallocate 3x3 matrix

for i = 1:Np
    for j = 1:Nv

        RMSE_matrix(i,j) = computeRMSE( ...
            prediction_val(i).Filename, ...
            validation_val(j).Filename);

    end
end

disp('RMSE Cross-Validation Matrix:');
disp(RMSE_matrix);

pred_names = {prediction_val.Signal};      % {'step12v_1','step12v_2','step12v_3'}
val_names  = {validation_val.Signal};

RMSE_Table = array2table(RMSE_matrix, ...
    'RowNames', pred_names, ...
    'VariableNames', val_names);

disp(RMSE_Table);
writetable(RMSE_Table, 'CrossValidation_RMSE.xlsx', ...
    'WriteRowNames', true);
%% ======= FUNCTION PART =======
function rmse = computeRMSE(predictions, validations)
    pred = load(predictions);
    val  = load(validations);

    pred_vals = pred.Signal;
    val_vals  = val.raw_data_shift;

    n = min(numel(pred_vals), numel(val_vals));
    pred_vals = pred_vals(1:n);
    val_vals  = val_vals(1:n);
    
    % Plot comparison between predicted and validation signals
    t = (0:n-1)'; % sample indices as x-axis
    figure;
    hold on;
plot(t, pred_vals, 'b-',  'LineWidth', 1.5);   % blue solid line
plot(t, val_vals,  'r--', 'LineWidth', 1.2);   % red dashed line
    hold off;
    grid on;
    xlabel('Sample Index');
    ylabel('Signal Value');
    title('Predicted vs. Validation Signal Comparison');
    legend('Predicted', 'Validation', 'Location', 'best');
    if n == 0
        rmse = NaN;
        return;
    end

    differences = pred_vals - val_vals;
    rmse = sqrt(mean(differences.^2));
end
