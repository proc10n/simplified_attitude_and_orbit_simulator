function [q_I2LVLH, omega_LVLH] = nadir_pointing(r_eci, v_eci, a_eci)
% Computes desired attitude quaternion and angular velocity for nadir pointing.
%
% Inputs:
%   r_eci  [3x1] - Position in ECI (m or km)
%   v_eci  [3x1] - Velocity in ECI
%   a_eci  [3x1] - Acceleration in ECI (including perturbations)
%
% Outputs:
%   q_I2LVLH    [4x1] - Desired quaternion (ECI -> Body=LVLH), scalar-first [q0 q1 q2 q3]
%   omega_LVLH  [3x1] - Desired angular velocity in LVLH frame (rad/s)

    %% Build LVLH basis vectors in ECI

    r_norm = norm(r_eci);
    h_vec  = cross(r_eci, v_eci);
    h_norm = norm(h_vec);

    % Nadir: +z_body = -r_hat
    e3 = -r_eci / r_norm;

    % -orbit normal
    e2 = -h_vec / h_norm;

    % Completes right-handed frame
    e1 = cross(e2, e3);

    %% Quaternion: ECI -> LVLH 
    C = [e1'; e2'; e3'];
    q_I2LVLH = dcm2quat(C);

    %% Angular velocity via Cdot * C'

    % Derivative of e3 = -r_hat
    r_dot = v_eci;
    de3 = -(r_dot - e3 * dot(e3, r_dot)) / r_norm;

    % Derivative of e2 = -h_hat
    h_dot = cross(r_eci, a_eci);
    de2 = -(h_dot - e2 * dot(e2, h_dot)) / h_norm;

    % Derivative of e1 = e2 x e3
    de1 = cross(de2, e3) + cross(e2, de3);

    Cdot = [de1'; de2'; de3'];
    S = Cdot * C';

    omega_LVLH = [S(3,2); S(1,3); S(2,1)];

end

