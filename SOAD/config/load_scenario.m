function scn = load_scenario(const)
% LOAD_SCENARIO  Mission scenario definition.
%
%   scn = load_scenario(const)
%
%   Defines the simulation epoch, orbital elements, and ground target.
%
%   Inputs:
%       const - struct from load_constants()
%
%   Output:
%       scn - struct with fields:
%           .utc0       [1x6]   Epoch [Y M D h m s]
%           .orb        struct  Keplerian elements
%           .trgt_ecef  [3x1]   Ground target in ECEF [m]

    %% Scenario Selection

    scn.type = [2; 2; 1]; % first position selects the scenario (individual controller or detumbling + pointing)
                           % second position selects the control scenario (inertial, nadir, sun...)
                           % third position selects the controller (PD, LQR, SMC...)
                           
    

    %% Simulation epoch (UTC)

    scn.utc0 = [2000 01 01 12 00 00];           % [Y M D h m s]

    %% Orbit (500 km Sun-synchronous)

    scn.orb.RE   = const.earth.a_km;             % [km] Earth radius
    scn.orb.a    = scn.orb.RE + 500;             % [km] Semi-major axis
    scn.orb.e    = 0.0;                          % [-]  Eccentricity
    scn.orb.i    = deg2rad(98.4);                % [rad] Inclination
    scn.orb.RAAN = deg2rad(62.6);                % [rad] RAAN
    scn.orb.w    = deg2rad(93.0);                % [rad] Argument of perigee
    scn.orb.v0   = deg2rad(100.0);               % [rad] True anomaly at epoch

    scn.orb.n    = [-sin(scn.orb.i)*sin(scn.orb.w); ...
                     sin(scn.orb.i)*cos(scn.orb.w); ...
                     cos(scn.orb.i)];             % [-] Orbit normal

    %% Ground target (São Paulo)

    lat = deg2rad(-23.5475);
    lon = deg2rad(-46.636111);
    h   = 0;

    N = const.earth.a / sqrt(1 - const.earth.e2 * sin(lat)^2);

    scn.trgt_ecef = [(N + h) * cos(lat) * cos(lon);
                     (N + h) * cos(lat) * sin(lon);
                     ((1 - const.earth.e2) * N + h) * sin(lat)];  % [m] ECEF

end