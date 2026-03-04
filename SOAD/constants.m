%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 1) Earth (WGS-84 + EGM96)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Gravity ---
earth.mu    = 3.986004418e14;    % [m^3/s^2]
earth.mu_km = 398600.4418;       % [km^3/s^2]

muEarth = 398600.4418;                       % [km^3/s^2]

% --- Reference ellipsoid (WGS-84) ---
earth.a      = 6378137.0;        % [m] semi-major axis
earth.a_km   = 6378.137;         % [km]
earth.f_inv  = 298.257223563;    % [-] inverse flattening
earth.f      = 1/earth.f_inv;    % [-] flattening
earth.b      = earth.a*(1-earth.f); % [m] semi-minor axis
earth.b_km   = earth.b/1e3;      % [km]
earth.e2     = earth.f*(2-earth.f); % [-] first eccentricity squared

% --- Rotation ---
earth.omega      = 7.292115e-5;  % [rad/s] 
earth.T_sidereal = 2*pi/earth.omega; % [s]
earth.T_solar    = 86400;        % [s]

% --- Zonal harmonics (EGM96) ---
earth.C20 = -1.08262668355315e-3;
earth.C30 =  2.53265648533224e-6;
earth.C40 =  1.61962159136700e-6;

% Convert to Jn:
earth.J2  = -earth.C20;          %  1.08262668355315e-3
earth.J3  = -earth.C30;          % -2.53265648533224e-6
earth.J4  = -earth.C40;          % -1.61962159136700e-6

% --- Extra ---
earth.g0   = 9.80665;            % [m/s^2] standard gravity
earth.mass = 5.97216787e24;      % [kg]



