% ---- GMST (Astronomical Almanac / Vallado) ----------------------------
function [gmst0_deg, gmst0_rad] = gmst(JD_UTC)
JD_J2000 = 2451545.0;
T_UTC    = (JD_UTC - JD_J2000) / 36525.0;

gmst0_deg = 280.46061837 ...
          + 360.98564736629 * (JD_UTC - JD_J2000) ...
          + 0.000387933    * T_UTC^2 ...
          - T_UTC^3 / 38710000;

gmst0_deg = mod(gmst0_deg, 360.0);        % [deg], in [0, 360)
gmst0_rad = gmst0_deg * (pi / 180);       % [rad], in [0, 2π)
end