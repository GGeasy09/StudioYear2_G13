clear();
% Setup Paths
folderPath = 'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Save_Variable';
fileList = dir(fullfile(folderPath, '*.mat'));
numFiles = length(fileList);

rmseResults = zeros(numFiles, numFiles);
fileNames = {fileList.name};
modelName = 'Lab1_parameter_estimation_student';

load_system(modelName); 

for i = 1:numFiles
    % --- 1. Load Parameters (Motor stats) ---
    paramData = load(fullfile(folderPath, fileNames{i}));
    
    % Move them to Base Workspace for Simulink
    assignin('base', 'motor_B',   paramData.motor_B);
    assignin('base', 'motor_Eff', paramData.motor_Eff);
    assignin('base', 'motor_J',   paramData.motor_J);
    assignin('base', 'motor_Ke',  paramData.motor_Ke);
    
    for j = 1:numFiles
        % --- 2. Load Signal/Input Data ---
        testData = load(fullfile(folderPath, fileNames{j}));
        
        % Move signal params to Base Workspace (This fixes your error!)
        assignin('base', 'motor_L',     testData.motor_L);
        assignin('base', 'motor_R',     testData.motor_R);
        assignin('base', 'Mode',        testData.Mode);
        assignin('base', 'Step_Value',  testData.Step_Value);
        assignin('base', 'Ramp_Slope',  testData.Ramp_Slope);
        assignin('base', 'Chirmp_Time', testData.Chirmp_Time); % <--- Fixed
        assignin('base', 'Chirmp_Freq', testData.Chirmp_Freq); % <--- Fixed
        assignin('base', 'Sine_Freq',   testData.Sine_Freq);
        assignin('base', 'Stair_Wait',  testData.Stair_Wait);
        
        fprintf('Testing Params from %s on Data from %s...\n', fileNames{i}, fileNames{j});
        
        try
            % --- 3. Run Simulation ---
            out = sim(modelName);
            
            % --- 4. RMSE Calculation ---
            simSignal = out.Estimate_Result.signals.values;
            % Note: Ensure your .mat file contains 'Signal' or whatever you named actual data
            actualSignal = testData.velocity; 
            
            len = min(length(simSignal), length(actualSignal));
            mse = mean((actualSignal(1:len) - simSignal(1:len)).^2);
            rmseResults(i,j) = sqrt(mse);
            
        catch ME
            fprintf('Error in sim: %s\n', ME.message);
            rmseResults(i,j) = NaN; % Mark as failed
        end
    end
end

% --- 5. Generate Table ---
T = array2table(rmseResults, 'RowNames', fileNames, 'VariableNames', matlab.lang.makeValidName(fileNames));
disp(T);