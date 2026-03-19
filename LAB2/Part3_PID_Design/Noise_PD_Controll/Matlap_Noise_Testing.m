%% Basic Setup
clear; 
% Load your base parameters
run("C:\Users\ACER\Documents\GitHub\StudioYear2_G13\LAB2\Parameter_Pendilum_[don't_Edit]\Lab2_params_student.m");

%% Input Variable Section
kp_value = 0.4626;
ki_value = 0.004608;
kd_value = 0.18;
rad2deg = 180/pi;
Setpoint = 180;
Initial_point = 0;
T_Kd_Filter = 0.01;

% For parsim, just use the model name
simfile = 'Noise.slx'; 
load_system("Noise.slx");

% Define your methods here
% Method_Name = ["Continuous Model cutoff Frequency 100Hz", "พContinuous Model cutoff Frequency 10000Hz","Continuous Model cutoff Frequency with anti windup 1000Hz","Continuous Model cutoff Frequency with anti windup 10000Hz"];
Method_Name = ["Continuous Model cutoff Frequency with anti windup 100Hz","Continuous Model cutoff Frequency with anti windup 10000Hz"];
Control_Method = [8,11]; 
Noise_Powers    = [0.0001];
Sample_Times    = [ 0.0001,0.001,0.01];

%% Pre-allocation
num_methods = length(Control_Method);
num_noises = length(Noise_Powers);
num_samples = length(Sample_Times);

% Use 3D cell arrays to store data for EVERY combination: {method, noise_power, sample_time}
results_pos = cell(num_methods, num_noises, num_samples);
results_time = cell(num_methods, num_noises, num_samples);
results_effort = cell(num_methods, num_noises, num_samples);

%% --- Simulation Loop ---
for m = 1:num_methods
    current_method_description = Method_Name(m);
    
    % 1. Create a unique figure for each Control Method (comparing noises)
    fig_name = sprintf('Analysis: %s', current_method_description);
    figure('Name', fig_name, 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.7]);
    
    ax1 = subplot(2, 1, 1); hold on; grid on; 
    title(ax1, ['Position: ', char(current_method_description)]);
    
    ax2 = subplot(2, 1, 2); hold on; grid on; 
    title(ax2, ['Control Effort: ', char(current_method_description)]);
    
    Controller_Method = Control_Method(m); % Set for Simulink
    
    for n = 1:num_noises
        Noise_Power = Noise_Powers(n); 
        
        for s = 1:num_samples
            Noise_Frequency = Sample_Times(s);
            
            % --- Run Simulation ---
            simout = sim(simfile);
            
            % --- Extract Data ---
            t = simout.Position_Data.Time;
            pos = simout.Position_Data.Data * rad2deg;
            effort = simout.control_effort.Data;
            
            % --- Store Data for later comparison ---
            results_pos{m, n, s} = pos;
            results_time{m, n, s} = t;
            results_effort{m, n, s} = effort;
            
            % --- Plotting inside Method-specific figure ---
            lbl = sprintf('Noise Pwr: %g, Freq: %g Hz', Noise_Power, 1/Noise_Frequency);
            
            plot(ax1, t, pos, 'DisplayName', lbl);
            plot(ax2, t, effort, 'DisplayName', lbl);
        end
    end
    
    % Formatting for the Method-specific figure
    yline(ax1, Setpoint, 'k:', 'Setpoint', 'HandleVisibility', 'off');
    ylabel(ax1, 'Degrees');
    legend(ax1, 'show', 'Location', 'eastoutside', 'FontSize', 7);
    
    yline(ax2, 12, 'r--', 'Max Voltage', 'HandleVisibility', 'off');
    yline(ax2, -12, 'r--', 'Min Voltage', 'HandleVisibility', 'off');
    ylabel(ax2, 'Volts');
    xlabel(ax2, 'Time (s)');
    legend(ax2, 'show', 'Location', 'eastoutside', 'FontSize', 7);
end

%% --- Cross-Method Comparison Figures (Separated by Noise Condition) ---
for n = 1:num_noises
    Noise_Power = Noise_Powers(n);
    
    for s = 1:num_samples
        Noise_Frequency = Sample_Times(s);
        
        % Create a label for the current noise condition to use in titles
        condition_label = sprintf('Noise Power: %g | Frequency: %g Hz', Noise_Power, 1/Noise_Frequency);
        
        % --- Figure: Position Comparison for this Noise Condition ---
        fig_title_pos = sprintf('Position Comparison (Freq %g Hz)', 1/Noise_Frequency);
        figure('Name', fig_title_pos, 'Color', 'w');
        hold on; grid on;
        
        for m = 1:num_methods
            % Plot each method for this specific noise condition
            plot(results_time{m, n, s}, results_pos{m, n, s}, 'LineWidth', 1.5, ...
                'DisplayName', char(Method_Name(m)));
        end
        yline(Setpoint, 'k:', 'Setpoint', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xlabel('Time (s)');
        ylabel('Position (Degrees)');
        title({'System Response Comparison', condition_label});
        legend('show', 'Location', 'southeast');
        
        % --- Figure: Control Effort Comparison for this Noise Condition ---
        fig_title_eff = sprintf('Effort Comparison (Freq %g Hz)', 1/Noise_Frequency);
        figure('Name', fig_title_eff, 'Color', 'w');
        hold on; grid on;
        
        for m = 1:num_methods
            plot(results_time{m, n, s}, results_effort{m, n, s}, 'LineWidth', 1.5, ...
                'DisplayName', char(Method_Name(m)));
        end
        yline(12, 'r--', 'Max Voltage (+12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
        yline(-12, 'r--', 'Min Voltage (-12V)', 'LineWidth', 1.2, 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
        xlabel('Time (s)');
        ylabel('Control Effort (Volts)');
        title({'Controller Effort Comparison', condition_label});
        legend('show', 'Location', 'northeast');
    end
end