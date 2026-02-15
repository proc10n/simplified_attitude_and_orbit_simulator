%% GMST at simulation epoch (UTC approximation)
%
%  Uses the IAU simplified formula with UTC in place of UT1.
%  Error vs true GMST: < 0.9 s of arc — negligible for CubeSat work.

% ---- Simulation epoch (UTC) -------------------------------------------
utc0 = [2000 01 01 12 00 00];   % [Y M D h m s]

% ---- Gregorian -> Julian Date ----------------------------------------
Y  = utc0(1);  Mo = utc0(2);  D  = utc0(3);
h  = utc0(4);  mi = utc0(5);  s  = utc0(6);

if Mo <= 2
    Y  = Y  - 1;
    Mo = Mo + 12;
end

A  = floor(Y / 100);
B  = 2 - A + floor(A / 4);

JD_UTC = floor(365.25 * (Y + 4716)) ...
       + floor(30.6001 * (Mo + 1)) ...
       + D + B - 1524.5 ...
       + (h + mi/60 + s/3600) / 24;

% ---- GMST (Astronomical Almanac / Vallado) ----------------------------

JD_J2000 = 2451545.0;
T_UTC    = (JD_UTC - JD_J2000) / 36525.0;

gmst0_deg = 280.46061837 ...
          + 360.98564736629 * (JD_UTC - JD_J2000) ...
          + 0.000387933    * T_UTC^2 ...
          - T_UTC^3 / 38710000;

gmst0_deg = mod(gmst0_deg, 360.0);        % [deg], in [0, 360)
gmst0_rad = gmst0_deg * (pi / 180);       % [rad], in [0, 2π)





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 1) Initial Conditions (Attitude + Orbit)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Attitude ---
w0 = [0; 0; 0];                                      % [rad/s]
q0 = [0.989288; 0.094060; 0.078926; 0.094060];       % [n.u.] scalar-first
q0 = [1; 0; 0; 0];
q0 = q0 / norm(q0);                                  % enforce unit quaternion

% -----------------------------
% Orbit
% -----------------------------
orb.KOE.RE    = 6378;                        % Earth radius [km]
orb.KOE.a     = orb.KOE.RE + 500;            % Semi-major axis [km]
orb.KOE.e     = 0.0;                         % Eccentricity
orb.KOE.i     = deg2rad(98.4);               % Inclination [rad]
orb.KOE.RAAN  = deg2rad(62.6);               % RAAN [rad]
orb.KOE.w     = deg2rad(93.0);               % Argument of perigee [rad]
orb.KOE.v0    = deg2rad(100.0);              % True anomaly at epoch [rad]

muEarth = 398600.4418;                       % [km^3/s^2]

% Convert KOE -> ECI
[r0_I, v0_I] = koe2eci( ...
    orb.KOE.a, orb.KOE.e, orb.KOE.i, ...
    orb.KOE.RAAN, orb.KOE.w, orb.KOE.v0, muEarth);

r0_I = r0_I * 1e3;
v0_I = v0_I * 1e3;

% Display
fprintf('r_ECI [m]   = [%.6f, %.6f, %.6f]\n', r0_I(1), r0_I(2), r0_I(3));
fprintf('v_ECI [m/s] = [%.6f, %.6f, %.6f]\n', v0_I(1), v0_I(2), v0_I(3));

% ------------------------------------------------------------
% Local function: Classical orbital elements to ECI state
% ------------------------------------------------------------
function [r_eci, v_eci] = koe2eci(a, e, i, RAAN, w, v, mu)
    % Semi-latus rectum
    p = a * (1 - e^2);

    % Radius magnitude
    r = p / (1 + e*cos(v));

    % State in perifocal frame (PQW)
    r_pqw = [r*cos(v); r*sin(v); 0];
    v_pqw = sqrt(mu/p) * [-sin(v); e + cos(v); 0];

    % Rotation PQW -> ECI: R3(RAAN)*R1(i)*R3(w)
    Q_pqw_to_eci = R3(RAAN) * R1(i) * R3(w);

    % ECI vectors
    r_eci = Q_pqw_to_eci * r_pqw;
    v_eci = Q_pqw_to_eci * v_pqw;
end

function R = R1(theta)
    c = cos(theta); s = sin(theta);
    R = [1  0  0;
         0  c -s;
         0  s  c];
end

function R = R3(theta)
    c = cos(theta); s = sin(theta);
    R = [ c -s  0;
          s  c  0;
          0  0  1];
end
