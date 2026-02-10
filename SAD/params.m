%% Parameters
% Last Change: 27/10/2025

% Satellite

J = [1 0 0;
     0 1 0;
     0 0 1];

% Motor

b =  1;  % Viscuous Friction
Kt = 1;  % Torque Constant
Ke = 1;  % Voltage Constant
R =  1;  % Armature Resistance
L =  1;  % Armature Inductance
Jw = 1;  % Motor Inertia

% Controller

omega_n = 0.32; % Natural Frequency
zeta    = 0.707; % Damping Ratio

Kp      = 2 * omega_n^2 * J;        % Proportional Gain
Kd      = 2 * omega_n * zeta * J;   % Derivative Gain

