% 1. Define your exact input data
T_sample = [2.417738, 2.417738, 2.393316, 2.368895, 2.344473, 2.320051, 2.295630, 2.271208, 0.170951];
Distance_Angle = [6.220353, 6.220353, 6.157522, 6.094690, 6.031858, 5.969026, 5.906194, 5.843362, 0.439823];

frame_time = 0.01;
accel_time_desired = 0.5;

num_moves = length(T_sample);

% Setup figure for visualization
figure('Name', 'Trapezoidal Velocity Profiles', 'Color', 'w');

for i = 1:num_moves
    T = T_sample(i);
    D = Distance_Angle(i);
    
    % 2. Handle the edge case: check if there is enough time to reach constant velocity
    if T < 2 * accel_time_desired
        ta = T / 2; % Fallback to a triangular profile
        fprintf('Move %d: Total time (%.3fs) is too short for 0.5s accel. Using triangular profile (ta = %.3fs).\n', i, T, ta);
    else
        ta = accel_time_desired;
    end
    
    % 3. Calculate kinematics
    v_max = D / (T - ta);
    a = v_max / ta;
    
    % 4. Create time vector for this move
    t = 0:frame_time:T;
    % Ensure the final frame strictly includes the exact end time 'T'
    if t(end) < T
        t = [t, T];
    end
    
    % Preallocate arrays for speed and memory
    q   = zeros(size(t)); % Position / Angle
    v   = zeros(size(t)); % Velocity
    acc = zeros(size(t)); % Acceleration
    
    % 5. Generate the profile
    for k = 1:length(t)
        tk = t(k);
        
        if tk <= ta
            % Phase 1: Acceleration
            q(k)   = 0.5 * a * tk^2;
            v(k)   = a * tk;
            acc(k) = a;
            
        elseif tk <= T - ta
            % Phase 2: Constant Velocity
            q(k)   = (0.5 * a * ta^2) + v_max * (tk - ta);
            v(k)   = v_max;
            acc(k) = 0;
            
        else
            % Phase 3: Deceleration
            t_dec  = tk - (T - ta);
            % Position = distance at start of decel + distance covered during decel
            q(k)   = D - 0.5 * a * (T - tk)^2; 
            v(k)   = v_max - a * t_dec;
            acc(k) = -a;
        end
    end
    
    % Force the final points to exactly match targets to avoid floating point drift
    q(end)   = D;
    v(end)   = 0;
    acc(end) = 0;
    
    % 6. Plot the results
    subplot(3,1,1); hold on; grid on;
    plot(t, q, 'LineWidth', 1.5);
    ylabel('Position'); title('Kinematic Profiles');
    
    subplot(3,1,2); hold on; grid on;
    plot(t, v, 'LineWidth', 1.5);
    ylabel('Velocity');
    
    subplot(3,1,3); hold on; grid on;
    plot(t, acc, 'LineWidth', 1.5);
    ylabel('Acceleration'); xlabel('Time (seconds)');
end