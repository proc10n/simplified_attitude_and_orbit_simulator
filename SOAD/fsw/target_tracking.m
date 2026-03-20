function [q_trgt, w_trgt] = target_tracking(gmst_deg, trgt_ecef, pos_ECI, vel_ECI, omegaE)

% -------- ECEF -> ECI (from GMST) --------
[trgt_eci, Cdot_ECI_ECEF] = ecef2eci(trgt_ecef, gmst_deg, omegaE);

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

% y-body = LOS x velocity 
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

% DCM: ECI -> Body 
C_B_ECI = [x_b'; y_b'; z_b'];

% -------- Quaternion from DCM (Shepperd) --------
q_trgt = dcm2quat(C_B_ECI);

if q_trgt(1) < 0
    q_trgt = -q_trgt;
end
q_trgt = q_trgt / norm(q_trgt);

% -------- Desired angular velocity --------
rho_dot_ECI = Cdot_ECI_ECEF * trgt_ecef - v_ECI;
w_ECI = cross(rho_ECI, rho_dot_ECI) / dot(rho_ECI, rho_ECI);
w_trgt = C_B_ECI * w_ECI;  % express in body frame

end