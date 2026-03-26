function [q_sun, omega_sun] = sun_pointing(r_eci, sun_eci)
% SUN_POINTING  Desired attitude for sun pointing via eigen-axis rotation.
%
%   [q_sun, omega_sun] = sun_pointing(r_eci, sun_eci)
%
%   Computes the ECI-to-Body quaternion that aligns body-z with the
%   satellite-to-sun direction. Roll about the sun line is unconstrained;
%   omega_sun is set to zero so the PD derivative term damps any residual
%   rotation about the pointing axis.
%
%   Inputs:
%       r_eci    [3x1] - Spacecraft position in ECI [m]
%       sun_eci  [3x1] - Sun position in ECI [m]
%
%   Outputs:
%       q_sun      [4x1] - Desired quaternion (ECI -> Body), scalar-first
%       omega_sun  [3x1] - Desired angular velocity in body frame [rad/s]

    %% LOS direction (satellite -> sun)
    rho = sun_eci - r_eci;
    s   = rho / norm(rho);

    %% Eigen-axis rotation: sun direction -> body-z
    b = [0; 0; 1];
    c = dot(s, b);

    if c < -1 + 1e-8
        % Anti-parallel: 180 deg rotation about body-x
        q_sun = [0; 1; 0; 0];
    elseif c > 1 - 1e-8
        % Already aligned
        q_sun = [1; 0; 0; 0];
    else
        ax    = cross(b, s);
        ax    = ax / norm(ax);
        theta = acos(c);
        q_sun = [cos(theta/2); sin(theta/2) * ax];
    end

    if q_sun(1) < 0
        q_sun = -q_sun;
    end

    omega_sun = [0; 0; 0];

end