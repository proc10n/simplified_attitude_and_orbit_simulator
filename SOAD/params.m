%% Parameters
% Last Change: 03/03/2026

% Satellite

J = [1 0 0;
     0 1 0;
     0 0 1];

% Motor (Portescap 20ECF14 8B 32, 12V variant)
% Source: 20ECF datasheet, V012024

b  = 9.5e-7;    % Viscous Friction       [Nm·s/rad] (estimated from no-load conditions)
Kt = 13.3e-3;   % Torque Constant         [Nm/A]
Ke = 13.3e-3;   % Back-EMF Constant       [V·s/rad]
R  = 4.61;      % Resistance (phase-phase) [Ohm]
L  = 0.4e-3;    % Inductance (phase-phase) [H]
Jw = 5e-6;      % Rotor Inertia            [kg·m^2] 

% Controller

omega_n = 0.32;  % Natural Frequency
zeta    = 0.707; % Damping Ratio

Kp      = 2 * omega_n^2 * J;        % Proportional Gain
Kd      = 2 * omega_n * zeta * J;   % Derivative Gain

