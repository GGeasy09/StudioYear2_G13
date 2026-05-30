% =========================================================================
% DC Motor Transfer Function & Control System Design (Updated Parameters)
% =========================================================================
clear; clc;

% 1. New motor parameters from your updated image
J  = 0.107e-1;           % Rotor inertia (.J = 0.107E-01f)
L  = 0.0002893301219;    % Motor inductance (.L = 0.0002893301219f)
R  = 0.4821323156;       % Motor resistance (.R = 0.4821323156f)
B  = 0.995;              % Viscous friction coefficient (.B = 0.995f)
Kt = 0.20906532;         % Torque constant (.kt = 0.20906532f)
Km = 0.2316;             % Back EMF constant (.km = 0.2316f)

% 2. Forward Transfer Function: sys_tf(s) = Velocity / Voltage
num_sys = Kt;
den_sys = [J*L, (J*R + B*L), (B*R + Kt*Km)];

sys_tf = tf(num_sys, den_sys);
disp('Forward Transfer Function sys_tf(s) [Input: Voltage, Output: Velocity]:')
sys_tf

% 3. Launch Control System Designer using Root Locus configuration
disp('Launching Control System Designer with Root Locus editor...');
controlSystemDesigner("rlocus", sys_tf);