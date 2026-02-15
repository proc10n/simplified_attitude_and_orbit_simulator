%% Sanity check for q_trgt

% --- Setup: use your actual sim parameters ---
mu = 3.986004418e14;  % m^3/s^2

% Pick a known state (or grab from your sim at some time step)
r_ECI = [6878e3; 0; 0];          % example: satellite on x-axis
v_ECI = [0; sqrt(mu/6878e3); 0]; % circular orbit velocity

% Target (São Paulo in ECEF, from your setup)
trgt_ecef = [(4.0169e6); (-4.2531e6); (-2.5324e6)];

% GMST at your epoch (adjust to match your sim)
gmst_deg = 280.46;  % example value
omegaE = 7.2921159e-5;

% --- Call your function ---
[q_trgt, w_trgt] = quatCmd(gmst_deg, trgt_ecef, r_ECI, v_ECI, omegaE);

% --- Verify ---
% Build DCM from q_trgt (scalar-first)
q0 = q_trgt(1); q1 = q_trgt(2); q2 = q_trgt(3); q3 = q_trgt(4);
C = [q0^2+q1^2-q2^2-q3^2,   2*(q1*q2+q0*q3),       2*(q1*q3-q0*q2);
     2*(q1*q2-q0*q3),         q0^2-q1^2+q2^2-q3^2,   2*(q2*q3+q0*q1);
     2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),         q0^2-q1^2-q2^2+q3^2];

% C is C_{Body/ECI}, so C' maps body vectors to ECI
z_body_in_ECI = C' * [0;0;1];

% Target in ECI
th = gmst_deg * pi/180;
C_ECI_ECEF = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
trgt_eci = C_ECI_ECEF * trgt_ecef;

% LOS direction
rho = trgt_eci - r_ECI;
rho_hat = rho / norm(rho);

fprintf('Body z in ECI:  [%.6f, %.6f, %.6f]\n', z_body_in_ECI);
fprintf('LOS in ECI:     [%.6f, %.6f, %.6f]\n', rho_hat);
fprintf('Dot product:     %.6f  (should be ~1.0)\n', dot(z_body_in_ECI, rho_hat));

%% Deeper diagnostics

% --- Call quatCmd to get intermediates ---
% (copy the internals or just recompute here)

th = gmst_deg * pi/180;
C_ECI_ECEF = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
trgt_eci = C_ECI_ECEF * trgt_ecef;

r_norm = norm(r_ECI);
h_ECI = cross(r_ECI, v_ECI);
h_norm = norm(h_ECI);

eR = -r_ECI / r_norm;
eW = -h_ECI / h_norm;
eS = cross(eW, eR);
eS = eS / norm(eS);
eW = cross(eR, eS);
eW = eW / norm(eW);

C_LVLH_ECI = [eS'; eW'; eR'];

% Check 1: Does q_LVLH_ECI actually match the DCM?
q0 = q_trgt(1); q1 = q_trgt(2); q2 = q_trgt(3); q3 = q_trgt(4);

fprintf('\n--- LVLH frame check ---\n');
fprintf('eS (should be ~velocity dir): [%.4f, %.4f, %.4f]\n', eS);
fprintf('eW (should be ~-orbit normal): [%.4f, %.4f, %.4f]\n', eW);
fprintf('eR (should be ~nadir):         [%.4f, %.4f, %.4f]\n', eR);

% Check 2: LOS in LVLH
rho_ECI = trgt_eci - r_ECI;
rho_LVLH = C_LVLH_ECI * rho_ECI;
u_los = rho_LVLH / norm(rho_LVLH);
fprintf('\n--- LOS in LVLH ---\n');
fprintf('u_los: [%.4f, %.4f, %.4f]\n', u_los);

% Check 3: What does q_los_LVLH do?
% If u_los = [0;0;1], q_los should be identity
% The angle between [0;0;1] and u_los:
angle_deg = acosd(min(1, max(-1, u_los(3))));
fprintf('Off-nadir angle: %.2f deg\n', angle_deg);

% Check 4: Test LVLH quaternion alone (without LOS rotation)
% If q_trgt = identity_los ⊗ q_LVLH_ECI, body z should point to nadir
% i.e. C_LVLH_ECI' * [0;0;1] should equal eR (nadir)
fprintf('\n--- Nadir check ---\n');
fprintf('C_LVLH'' * [0;0;1] = [%.4f, %.4f, %.4f]\n', C_LVLH_ECI' * [0;0;1]);
fprintf('eR (nadir):          [%.4f, %.4f, %.4f]\n', eR);
fprintf('These should match.\n');

% Check 5: Is the LVLH frame right-handed and orthonormal?
fprintf('\n--- Orthonormality ---\n');
fprintf('det(C_LVLH_ECI) = %.6f (should be 1)\n', det(C_LVLH_ECI));
fprintf('eS.eW = %.2e, eS.eR = %.2e, eW.eR = %.2e (should be ~0)\n', ...
    dot(eS,eW), dot(eS,eR), dot(eW,eR));

function [q_trgt, w_trgt] = quatCmd(gmst_deg, trgt_ecef, pos_ECI, vel_ECI, omegaE)

q_trgt = zeros(4,1);
w_trgt = zeros(3,1);

% -------- ECEF -> ECI (from GMST) --------
th = gmst_deg * (pi/180);
c  = cos(th);
s  = sin(th);

C_ECI_ECEF = [ c -s 0;
               s  c 0;
               0  0 1];

Cdot_ECI_ECEF = omegaE * [ -s -c 0;
                             c -s 0;
                             0  0 0 ];

trgt_ecef = trgt_ecef(:);
trgt_eci  = C_ECI_ECEF * trgt_ecef;

% -------- Inputs --------
r_ECI = pos_ECI(:);
v_ECI = vel_ECI(:);

if ~all(isfinite([r_ECI; v_ECI; trgt_ecef; gmst_deg; omegaE]))
    q_trgt = [1;0;0;0];
    w_trgt = [0;0;0];
    return;
end

% -------- LOS vector --------
rho_ECI  = trgt_eci - r_ECI;
rho_norm = norm(rho_ECI);

if (~all(isfinite(rho_ECI))) || (rho_norm <= eps)
    q_trgt = [1;0;0;0];
    w_trgt = [0;0;0];
    return;
end

% -------- Build target DCM directly in ECI --------
% z-body = LOS direction
z_b = rho_ECI / rho_norm;

% y-body = LOS x velocity (constrains roll)
y_b = cross(z_b, v_ECI);
ny = norm(y_b);
if ny < 1e-12
    y_b = cross(z_b, [0;0;1]);
    ny = norm(y_b);
end
y_b = y_b / ny;

% x-body completes right-handed frame
x_b = cross(y_b, z_b);
x_b = x_b / norm(x_b);

% DCM: ECI -> Body (rows are body axes in ECI)
C_B_ECI = [x_b'; y_b'; z_b'];

% -------- Quaternion from DCM (Shepperd) --------
tr = C_B_ECI(1,1) + C_B_ECI(2,2) + C_B_ECI(3,3);
q0 = 0.0; q1 = 0.0; q2 = 0.0; q3 = 0.0;

vals = [tr, C_B_ECI(1,1), C_B_ECI(2,2), C_B_ECI(3,3)];
[~, idx] = max(vals);

switch idx
    case 1
        q0 = 0.5 * sqrt(1 + tr);
        k = 0.25 / q0;
        q1 = (C_B_ECI(2,3) - C_B_ECI(3,2)) * k;
        q2 = (C_B_ECI(3,1) - C_B_ECI(1,3)) * k;
        q3 = (C_B_ECI(1,2) - C_B_ECI(2,1)) * k;
    case 2
        q1 = 0.5 * sqrt(1 + 2*C_B_ECI(1,1) - tr);
        k = 0.25 / q1;
        q0 = (C_B_ECI(2,3) - C_B_ECI(3,2)) * k;
        q2 = (C_B_ECI(1,2) + C_B_ECI(2,1)) * k;
        q3 = (C_B_ECI(3,1) + C_B_ECI(1,3)) * k;
    case 3
        q2 = 0.5 * sqrt(1 + 2*C_B_ECI(2,2) - tr);
        k = 0.25 / q2;
        q0 = (C_B_ECI(3,1) - C_B_ECI(1,3)) * k;
        q1 = (C_B_ECI(1,2) + C_B_ECI(2,1)) * k;
        q3 = (C_B_ECI(2,3) + C_B_ECI(3,2)) * k;
    case 4
        q3 = 0.5 * sqrt(1 + 2*C_B_ECI(3,3) - tr);
        k = 0.25 / q3;
        q0 = (C_B_ECI(1,2) - C_B_ECI(2,1)) * k;
        q1 = (C_B_ECI(3,1) + C_B_ECI(1,3)) * k;
        q2 = (C_B_ECI(2,3) + C_B_ECI(3,2)) * k;
end

q_trgt = [q0; q1; q2; q3];
if q_trgt(1) < 0
    q_trgt = -q_trgt;
end
q_trgt = q_trgt / norm(q_trgt);

% -------- Desired angular velocity --------
rho_dot_ECI = Cdot_ECI_ECEF * trgt_ecef - v_ECI;
w_ECI = cross(rho_ECI, rho_dot_ECI) / dot(rho_ECI, rho_ECI);
w_trgt = C_B_ECI * w_ECI;  % express in body frame

end