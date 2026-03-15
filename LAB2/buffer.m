clear;
% 1. Define the data from the image
true_temperatures = [50.005, 49.994, 49.993, 50.001, 50.006, ...
                     49.998, 50.021, 50.005, 50, 49.997];

measurements = [49.986, 49.963, 50.09, 50.001, 50.018, ...
                50.05, 49.938, 49.858, 49.965, 50.114];

% 2. Create index vector for iterations
iterations = 1:length(true_temperatures);

% 3. Combine into a matrix
% Each row represents a variable (True Temp and Measurement)
data_matrix = [true_temperatures; measurements];

% 4. Create UNIQUE column names: Iteration_1, Iteration_2, etc.
% The %d placeholder inserts the actual number into the string
col_names = arrayfun(@(x) sprintf('Iteration_%d', x), iterations, 'UniformOutput', false);

% 5. Create the table with RowNames
RowTable = array2table(data_matrix, ...
    'VariableNames', col_names, ...
    'RowNames', {'True_Temp_C', 'Measurement_C'});

% Display the table
disp('--- Temperature Data (Row Table Format) ---');
disp(RowTable);