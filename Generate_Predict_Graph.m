%% load every parameter into block
clear;
fuking_universe('step12v_1', 7.55E-05, 0.6496846103, 0.009105944233, 4.98E-07, ...
    'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1 Part2 Parameter\Step_12V.mat');

fuking_universe('step12v_2', 8.03E-05, 0.6370546747, 0.01020231143, 2.55E-07, ...
    'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1 Part2 Parameter\Step_12V.mat');

fuking_universe('step12v_3', 7.18E-05, 0.6642138794, 0.008288779191, 2.65E-07, ...
    'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1 Part2 Parameter\Step_12V.mat');


%% Universal function (fixed version)
function loading = fuking_universe(filename, B, eff, J, ke, parameter)
    folderPath = 'C:\Users\ACER\Documents\GitHub\StudioYear2_G13\Lab1 Part2 Data\Lab1 Part2 Data Predicted Val';
    fullFile = fullfile(folderPath, [filename, '.mat']);
    
    % 1. Create a Simulation Input object
    modelName = 'Lab1_parameter_estimation_student';
    simIn = Simulink.SimulationInput(modelName);
    
    % 2. Load the .mat file parameters into the simulation's model workspace
    vars = load(parameter);
    fn = fieldnames(vars);
    for k = 1:numel(fn)
        simIn = simIn.setVariable(fn{k}, vars.(fn{k}));
    end
    
    % 3. Set the specific motor parameters for THIS run
    simIn = simIn.setVariable('motor_R', 3.341);
    simIn = simIn.setVariable('motor_L', 0.02587);
    simIn = simIn.setVariable('motor_B', B);
    simIn = simIn.setVariable('motor_Eff', eff);
    simIn = simIn.setVariable('motor_Ke', ke);
    simIn = simIn.setVariable('motor_J', J);
    
    % 4. Run the simulation
    out = sim(simIn); 
    
    % 5. Extract data
    Signal = out.Estimate_Result.signals.values;
    time   = out.tout;
    
    % Plot and Save
    figure; plot(time, Signal); title(filename);
    save(fullFile, "Signal", "time");
    loading = fullFile;
end
