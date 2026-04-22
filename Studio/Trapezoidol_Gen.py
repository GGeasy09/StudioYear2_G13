import numpy as np
import matplotlib.pyplot as plt

# 1. Define your exact input data
T_sample = [2.417738, 2.417738, 2.393316, 2.368895, 2.344473, 2.320051, 2.295630, 2.271208, 0.170951]
Distance_Angle = [6.22, 6.22, 6.16, 6.09, 6.03, 5.97, 5.91, 5.84, 0.44]

frame_time = 0.01
accel_time_desired = 0.5
num_moves = len(T_sample)

# Global trackers to hold the concatenated arrays
global_t = []
global_q = []
global_v = []
global_acc = []

# Trackers for the current offset
current_time = 0.0
current_pos = 0.0

for i in range(num_moves):
    T = T_sample[i]
    D = Distance_Angle[i]
    
    # 2. Handle the edge case
    if T < 2 * accel_time_desired:
        ta = T / 2.0 
        print(f"Move {i+1}: Total time ({T:.3f}s) is too short for 0.5s accel. Using triangular profile (ta = {ta:.3f}s).")
    else:
        ta = accel_time_desired
        
    # 3. Calculate kinematics for the current segment
    v_max = D / (T - ta)
    a = v_max / ta
    
    # 4. Create local time vector
    t = np.arange(0, T, frame_time)
    if len(t) == 0 or t[-1] < T:
        t = np.append(t, T)
        
    q   = np.zeros(len(t))
    v   = np.zeros(len(t))
    acc = np.zeros(len(t))
    
    # 5. Generate the local profile
    for k in range(len(t)):
        tk = t[k]
        if tk <= ta:
            q[k]   = 0.5 * a * tk**2
            v[k]   = a * tk
            acc[k] = a
        elif tk <= T - ta:
            q[k]   = (0.5 * a * ta**2) + v_max * (tk - ta)
            v[k]   = v_max
            acc[k] = 0
        else:
            t_dec  = tk - (T - ta)
            q[k]   = D - 0.5 * a * (T - tk)**2 
            v[k]   = v_max - a * t_dec
            acc[k] = -a
            
    # Force exact endpoints
    q[-1]   = D
    v[-1]   = 0
    acc[-1] = 0
    
    # Remove the very first point for moves after the first one 
    # This prevents duplicate timestamps where segments connect (e.g., end of Move 1 and start of Move 2)
    if i > 0:
        t = t[1:]
        q = q[1:]
        v = v[1:]
        acc = acc[1:]
        
    # Shift local arrays by the global offsets and append to the main lists
    global_t.extend(t + current_time)
    global_q.extend(q + current_pos)
    global_v.extend(v)
    global_acc.extend(acc)
    
    # Update offsets for the next segment
    current_time += T
    current_pos += D

# 6. Plot the continuous results
fig, axs = plt.subplots(3, 1, figsize=(12, 8))
fig.canvas.manager.set_window_title('Continuous Trapezoidal Velocity Profile')
fig.patch.set_facecolor('white')

axs[0].plot(global_t, global_q, linewidth=1.5, color='b')
axs[0].set_ylabel('Position')
axs[0].set_title('Continuous Kinematic Profile')
axs[0].grid(True)

axs[1].plot(global_t, global_v, linewidth=1.5, color='g')
axs[1].set_ylabel('Velocity')
axs[1].grid(True)

# Note: Acceleration will show sharp jumps to zero between moves since the robot stops at each point
axs[2].plot(global_t, global_acc, linewidth=1.5, color='r')
axs[2].set_ylabel('Acceleration')
axs[2].set_xlabel('Time (seconds)')
axs[2].grid(True)

plt.tight_layout()
plt.show()