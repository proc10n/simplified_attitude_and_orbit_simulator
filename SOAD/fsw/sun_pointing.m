function [q_sun, omega_sun] = sun_pointing(r_eci, sun_eci)
% Computes desired attitude quaternion for sun pointing.
% Uses eigen-axis rotation to align body-z with the sun direction.
% Roll about the sun line is unconstrained; w_ref = 0 prevents drift.
%
% Inputs:
%   sun_eci  [3x1] - Sun direction in ECI (unit vector)
%   q_curr   [4x1] - Current attitude quaternion (ECI -> Body), scalar-first
%
% Outputs:
%   q_sun      [4x1] - Desired quaternion (ECI -> Body), scalar-first
%   omega_sun  [3x1] - Desired angular velocity in body frame [rad/s]

    rho = sun_eci - r_eci;
    s   = rho / norm(rho);
    b   = [0; 0; 1];
    c   = dot(s, b);

    if c < -1 + 1e-8
        q_sun = [0; 1; 0; 0];
    elseif c > 1 - 1e-8
        q_sun = [1; 0; 0; 0];
    else
        ax    = cross(s, b);
        ax    = ax / norm(ax);
        theta = acos(c);
        q_sun = [cos(theta/2); sin(theta/2) * ax];
    end

    if q_sun(1) < 0
        q_sun = -q_sun;
    end

    omega_sun = [0; 0; 0];
end