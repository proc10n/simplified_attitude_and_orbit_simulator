%% GMST at simulation epoch (UTC approximation)
%
%  Uses the IAU simplified formula with UTC in place of UT1.
%  Error vs true GMST: < 0.9 s of arc — negligible for CubeSat work.

% ---- Simulation epoch (UTC) -------------------------------------------
utc0 = [2026 02 09 00 00 00];   % [Y M D h m s]

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
%  θ_GMST = 280.46061837°
%         + 360.98564736629 * (JD - J2000)
%         + 0.000387933 * T²
%         - T³ / 38710000
%
%  where T = Julian centuries of UTC from J2000.
%  The linear term absorbs both the sidereal-day conversion and the
%  fractional-day contribution, so no separate 0h-UT1 split is needed.

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
% q0 = [1; 0; 0; 0];
q0 = q0 / norm(q0);                                  % enforce unit quaternion

% --- Orbit (ECI, two-body) ---
% Convention:
%   r0_I, v0_I are expressed in inertial frame I (ECI), SI units.
%   Use one active case below via orbit.ic = orbit.test.<name>.

% ============================================================
% Orbit test presets
% ============================================================

% 1) LEO circular equatorial, h = 500 km
orbit.test.LEO500_circ_eq.r0_I = [6878137.0; 0.0; 0.0];                 % [m]
orbit.test.LEO500_circ_eq.v0_I = [0.0; 7612.608173; 0.0];               % [m/s]
orbit.test.LEO500_circ_eq.note = 'e~0, i~0 deg, T~94.62 min';

% ============================================================
% Active orbit initial condition (pick one)
% ============================================================
orbit.ic = orbit.test.LEO500_circ_eq;

% Export active IC to variables used by Simulink
r0_I = orbit.ic.r0_I;    % [m]
v0_I = orbit.ic.v0_I;    % [m/s]
