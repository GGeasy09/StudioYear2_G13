clear();
% Setup Paths
folderPath = 'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Save_Variable';
fileList = dir(fullfile(folderPath, '*.mat'));
numFiles = length(fileList);

fileNames = {fileList.name};
modelName = 'Lab1_parameter_estimation_student';

load_system(modelName); 

load('Save_Variable - Copy\Sinepi.mat')
motor_B = 1.02E-04;
motor_Eff = 0.9997684157;
motor_J = 0.0002886642529;
motor_Ke = 0.0439752384;
    
    % % Move them to Base Workspace for Simulink
    % assignin('base', 'motor_B',   paramData.motor_B);
    % assignin('base', 'motor_Eff', paramData.motor_Eff);
    % assignin('base', 'motor_J',   paramData.motor_J);
    % assignin('base', 'motor_Ke',  paramData.motor_Ke);
    
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
        
        fprintf('Testing Params from %s on Data from %s...\n', 'Parameter_Estimator', fileNames{j});
        
        try
            % --- 3. Run Simulation ---
            out = sim(modelName);
            
            % --- 4. Plot actualsignal and sim signal ---
            simSignal = out.Estimate_Result.signals.values;
            % Note: Ensure your .mat file contains 'Signal' or whatever you named actual data
            actualSignal = testData.velocity;
            time = testData.time;
            figure;
            hold on;
            % plot(time, actualSignal, 'b', 'DisplayName', 'Actual Signal');
            plot(time, simSignal, 'r--', 'DisplayName', 'Simulated Signal');
            xlabel('Time (s)');
            ylabel('Signal Amplitude');
            title(sprintf('Comparison of Actual and Simulated Signals for %s', fileNames{j}));
            legend show;
            hold off;
        en
  end


