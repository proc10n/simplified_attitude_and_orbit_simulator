%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 1) Target 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lat = deg2rad(-23.5475); 
lon = deg2rad(-46.636111); % (Sao Paulo)

h = 0; 
N = earth.a/sqrt(1 - earth.e2*sin(lat)^2);

trgt_ecef = [(N+h)*cos(lat)*cos(lon); 
             (N+h)*cos(lat)*sin(lon); 
             ((1-earth.e2)*N+h)*sin(lat)];   % [m] ECEF, Ellipsoid

% ---- Simulation epoch (UTC) -------------------------------------------
utc0 = [2000 01 01 12 00 00];          % [Y M D h m s]

orb.RE    = 6378;                        % Earth radius [km]
orb.a     = orb.RE + 500;                % Semi-major axis [km]
orb.e     = 0.0;                         % Eccentricity
orb.i     = deg2rad(98.4);               % Inclination [rad]
orb.RAAN  = deg2rad(62.6);               % RAAN [rad]
orb.w     = deg2rad(93.0);               % Argument of perigee [rad]
orb.v0    = deg2rad(100.0);              % True anomaly at epoch [rad]

orb.n     = [-sin(orb.i)*sin(orb.w); sin(orb.i)*cos(orb.w); cos(orb.i)];