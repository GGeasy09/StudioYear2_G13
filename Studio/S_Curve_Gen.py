import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
title_name = "Robot Arm S-Curve Motion Profile"  # <--- Edit Title Here
Distance_Angle = [6.22, 6.22, 6.16, 6.09, 6.03, 5.97, 5.91, 5.84, 0.44]

# --- PARAMETERS ---
fixed_segment_time = 2.111  
tj = 0.5/3        # Jerk time (1/3 of accel time is the sweet spot)
ta = 0.5          # Total accel time
frame_time = 0.01
inertia = 0.5
# ------------------

global_t, global_v, global_acc = [], [], []
current_time = 0.0

print(f"{'Move':<6} | {'Max Vel':<10} | {'Max Torque':<10}")
print("-" * 35)

for i in range(len(Distance_Angle)):
    T = fixed_segment_time
    D = Distance_Angle[i]
    
    # Kinematics logic
    v_max = D / (T - ta)
    a_max = v_max / (ta - tj)
    j = a_max / tj
    
    print(f"{i+1:<6} | {v_max:<10.3f} | {inertia * a_max:<10.3f}")

    t = np.arange(0, T, frame_time)
    if t[-1] < T: t = np.append(t, T)
    
    v = np.zeros(len(t))
    acc = np.zeros(len(t))
    
    for k in range(len(t)):
        tk = t[k]
        # Acceleration Phase
        if tk <= tj:
            acc[k] = j * tk
            v[k] = 0.5 * j * tk**2
        elif tk <= ta - tj:
            acc[k] = a_max
            v[k] = 0.5 * j * tj**2 + a_max * (tk - tj)
        elif tk <= ta:
            acc[k] = a_max - j * (tk - (ta - tj))
            v[k] = v_max - 0.5 * j * (ta - tk)**2
        # Constant Velocity Phase
        elif tk <= T - ta:
            acc[k] = 0
            v[k] = v_max
        # Deceleration Phase (Mirror of Accel)
        else:
            td = tk - (T - ta)
            if td <= tj:
                acc[k] = -j * td
                v[k] = v_max - 0.5 * j * td**2
            elif td <= ta - tj:
                acc[k] = -a_max
                v[k] = v_max - (0.5 * j * tj**2 + a_max * (td - tj))
            else:
                tr = T - tk
                acc[k] = -j * tr
                v[k] = 0.5 * j * tr**2

    # Clean duplicates between segments
    if i > 0: 
        t, v, acc = t[1:], v[1:], acc[1:]
    
    global_t.extend(t + current_time)
    global_v.extend(v)
    global_acc.extend(acc)
    
    current_time += T

# --- PLOTTING ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
fig.suptitle(title_name, fontsize=14, fontweight='bold')

# Speed Profile
ax1.plot(global_t, global_v, color='forestgreen', lw=2)
ax1.set_ylabel('Speed (rad/s)')
ax1.grid(True, alpha=0.3)

# Acceleration Profile
ax2.plot(global_t, global_acc, color='crimson', lw=2)
ax2.set_ylabel('Acc (rad/s²)')
ax2.set_xlabel('Total Time (seconds)')
ax2.grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust for suptitle
plt.show()