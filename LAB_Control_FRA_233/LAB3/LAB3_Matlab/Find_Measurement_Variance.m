% 1. Load the data into struct 'S'
S = load("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB3\LAB3_Matlab_Result\measurement_varaince.mat");

% 2. Automatically get the name of the variable stored inside the struct
fields = fieldnames(S);
var_name = fields{1}; 

% 3. Extract the timeseries object
ts_object = S.(var_name);

% 4. Calculate the variance from the '.Data' property of the timeseries
% We include 'omitnan' to ensure any missing sensor readings don't break the math
measurement_variance = var(ts_object.Data, 'omitnan');

% 5. Display the result in the Command Window
fprintf('The variance of your timeseries data is: %f\n', measurement_variance);