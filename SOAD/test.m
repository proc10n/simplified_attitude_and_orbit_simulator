%% Root Locus: Motor Speed Loop
% Plant: Resistive DC motor model (L neglected)
% Controller: PI
%
% Open-loop: L(s) = C(s) * G(s)
%
% G(s) = Kt / (R*Jw_total*s + Kt*Ke + R*b)
% C(s) = Kp * (s + Ki/Kp) / s

clc; clear; close all;

%% Motor Parameters (Portescap 20ECF14 8B 32)

Kt = 13.3e-3;   % Torque Constant         [Nm/A]
Ke = 13.3e-3;   % Back-EMF Constant       [V·s/rad]
R  = 4.61;      % Resistance (phase-phase) [Ohm]
b  = 9.5e-7;    % Viscous Friction         [Nm·s/rad]
Jw = 3e-7;      % Rotor Inertia            [kg·m^2]
Jfw = 5e-6;     % Flywheel Inertia         [kg·m^2]

Jw_total = Jw + Jfw;

%% Plant Transfer Function (Voltage -> Speed)

num_G = Kt;
den_G = [R*Jw_total, (Kt*Ke + R*b)];

G = tf(num_G, den_G);

fprintf('--- Plant ---\n');
fprintf('Plant pole: %.2f rad/s\n', -den_G(2)/den_G(1));
fprintf('DC gain:    %.2f (rad/s)/V\n', dcgain(G));
fprintf('Mech. time constant: %.2f ms\n', den_G(1)/den_G(2)*1e3);

%% PI Controller

Kp = 1;
Ki = 30;

C = tf([Kp, Ki], [1, 0]);

fprintf('\n--- PI Controller ---\n');
fprintf('Kp = %.3f, Ki = %.3f\n', Kp, Ki);
fprintf('PI zero: %.2f rad/s\n', Ki/Kp);

%% Open-Loop and Root Locus

L = C * G;

figure('Name', 'Root Locus', 'Position', [100 100 900 600]);

subplot(1,2,1);
rlocus(G);
title('Plant only');
grid on;

subplot(1,2,2);
rlocus(L);
title('PI \times Plant');
grid on;

%% Closed-Loop Step Response

T = feedback(L, 1);

figure('Name', 'Step Response', 'Position', [100 750 600 400]);
step(T);
title('Closed-Loop Step Response (\Omega_{ref} \rightarrow \Omega)');
grid on;

fprintf('\n--- Closed-Loop ---\n');
fprintf('Poles: ');
disp(pole(T).');