import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
title_name = "Robot Arm Trapezoidal Motion Profile"  # <--- Edit Title Here
Distance_Angle = [6.22, 6.22, 6.16, 6.09, 6.03, 5.97, 5.91, 5.84, 0.44]

# --- PARAMETERS ---
fixed_segment_time = 2.111  
ta = 0.5          # Acceleration/Deceleration ramp time
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
    
    # 1. Trapezoidal Kinematics Logic
    # In trapezoidal, v_max = D / (T - ta)
    # Constant acceleration a = v_max / ta
    v_max = D / (T - ta)
    a_const = v_max / ta
    
    print(f"{i+1:<6} | {v_max:<10.3f} | {inertia * a_const:<10.3f}")

    t = np.arange(0, T, frame_time)
    if t[-1] < T: t = np.append(t, T)
    
    v = np.zeros(len(t))
    acc = np.zeros(len(t))
    
    for k in range(len(t)):
        tk = t[k]
        
        # Phase 1: Constant Acceleration
        if tk <= ta:
            acc[k] = a_const
            v[k] = a_const * tk
            
        # Phase 2: Constant Velocity (Cruise)
        elif tk <= T - ta:
            acc[k] = 0
            v[k] = v_max
            
        # Phase 3: Constant Deceleration
        else:
            acc[k] = -a_const
            v[k] = v_max - a_const * (tk - (T - ta))

    # Clean duplicates and prevent negative velocity from tiny float errors
    v = np.maximum(v, 0)
    if i > 0: 
        t, v, acc = t[1:], v[1:], acc[1:]
    
    global_t.extend(t + current_time)
    global_v.extend(v)
    global_acc.extend(acc)
    
    current_time += T

# --- PLOTTING ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
fig.suptitle(title_name, fontsize=14, fontweight='bold')

# Speed Profile (Trapezoids)
ax1.plot(global_t, global_v, color='forestgreen', lw=2)
ax1.set_ylabel('Speed (rad/s)')
ax1.grid(True, alpha=0.3)

# Acceleration Profile (Rectangular steps)
ax2.plot(global_t, global_acc, color='crimson', lw=2)
ax2.set_ylabel('Acc (rad/s²)')
ax2.set_xlabel('Total Time (seconds)')
ax2.grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()