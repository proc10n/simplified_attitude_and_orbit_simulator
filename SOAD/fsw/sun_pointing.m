function [q_sun, omega_sun] = sun_pointing(sun_eci, v_sun_eci, r_eci, v_eci, a_eci)
% Computes desired attitude quaternion and angular velocity for sun pointing.
%
% Inputs:
%   sun_eci    [3x1] - Sun position in ECI [m]
%   v_sun_eci  [3x1] - Sun velocity in ECI [m/s]
%   r_eci      [3x1] - Spacecraft position in ECI [m]
%   v_eci      [3x1] - Spacecraft velocity in ECI [m/s]
%   a_eci      [3x1] - Spacecraft acceleration in ECI [m/s^2]
%
% Outputs:
%   q_sun      [4x1] - Desired quaternion (ECI -> Body), scalar-first
%   omega_sun  [3x1] - Desired angular velocity in body frame [rad/s]

    %% LOS vector (satellite -> sun)
    rho      = sun_eci - r_eci;
    rho_norm = norm(rho);
    rho_dot  = v_sun_eci - v_eci;

    %% Build basis vectors in ECI
    % z-body = sun LOS direction
    e3 = rho / rho_norm;

    % y-body = LOS x LOS_dot
    y_raw = cross(e3, rho_dot);
    e2 = y_raw / norm(y_raw);

    % x-body completes right-handed frame
    e1 = cross(e2, e3);
    e1 = e1 / norm(e1);

    %% Quaternion
    C = [e1'; e2'; e3'];
    q_sun = dcm2quat(C);

    %% Angular velocity via Cdot * C'

    % Derivative of e3 (sun LOS unit vector)
    de3 = (rho_dot - e3 * dot(e3, rho_dot)) / rho_norm;

    % Derivative of rho_dot (a_sun ≈ 0)
    rho_ddot = -a_eci;

    % Derivative of y_raw = cross(e3, rho_dot)
    dy_raw = cross(de3, rho_dot) + cross(e3, rho_ddot);
    y_norm = norm(y_raw);
    de2 = (dy_raw - e2 * dot(e2, dy_raw)) / y_norm;

    % Derivative of e1 = e2 x e3
    de1 = cross(de2, e3) + cross(e2, de3);

    Cdot = [de1'; de2'; de3'];
    S = Cdot * C';

    omega_sun = [S(3,2); S(1,3); S(2,1)];

end