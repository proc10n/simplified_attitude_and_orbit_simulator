function const = load_constants()
% LOAD_CONSTANTS  Physical constants and reference models.
%
%   const = load_constants()
%
%   Returns a struct with Earth (WGS-84/EGM96), Sun, and physics constants.

    %% Earth (WGS-84 + EGM96)

    % Gravity
    const.earth.mu      = 3.986004418e14;       % [m^3/s^2]
    const.earth.mu_km   = 398600.4418;           % [km^3/s^2]

    % Reference ellipsoid (WGS-84)
    const.earth.a        = 6378137.0;            % [m]   Semi-major axis
    const.earth.a_km     = 6378.137;             % [km]
    const.earth.f_inv    = 298.257223563;        % [-]   Inverse flattening
    const.earth.f        = 1 / const.earth.f_inv;
    const.earth.b        = const.earth.a * (1 - const.earth.f);   % [m] Semi-minor axis
    const.earth.b_km     = const.earth.b / 1e3;
    const.earth.e2       = const.earth.f * (2 - const.earth.f);   % [-] First eccentricity squared

    % Rotation
    const.earth.omega      = 7.292115e-5;        % [rad/s]
    const.earth.T_sidereal = 2*pi / const.earth.omega;  % [s]
    const.earth.T_solar    = 86400;              % [s]

    % Zonal harmonics (EGM96)
    const.earth.C20 = -1.08262668355315e-3;
    const.earth.C30 =  2.53265648533224e-6;
    const.earth.C40 =  1.61962159136700e-6;

    const.earth.J2  = -const.earth.C20;          %  1.08262668355315e-3
    const.earth.J3  = -const.earth.C30;          % -2.53265648533224e-6
    const.earth.J4  = -const.earth.C40;          % -1.61962159136700e-6

    % Extra
    const.earth.g0   = 9.80665;                  % [m/s^2] Standard gravity
    const.earth.mass = 5.97216787e24;            % [kg]

    %% Sun

    const.sun.solar_const = 1361;                % [W/m^2] Solar constant

    %% Physics

    const.phys.c = 299792458;                    % [m/s] Speed of light

end