
JD_UTC = greg2jd(utc0);                % Gregorian --> JD

[gmst0_deg, gmst0_rad] = gmst(JD_UTC); % Obtain initial GMST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 1) Initial Conditions (Attitude + Orbit)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ----------------------------
% Attitude
% ----------------------------
w0 = [0.0; 0.0; 0.0];                                      % [rad/s]
q0 = [0.989288; 0.094060; 0.078926; 0.094060];       % [n.u.] scalar-first
% q0 = [1; 0; 0; 0];
q0 = q0 / norm(q0);                                  % enforce unit quaternion

% -----------------------------
% Orbit
% -----------------------------
% Convert KOE -> ECI
[r0_I, v0_I] = koe2eci( ...
    orb.a, orb.e, orb.i, ...
    orb.RAAN, orb.w, orb.v0, muEarth);

r0_I = r0_I * 1e3;
v0_I = v0_I * 1e3;

% Display
% fprintf('r_ECI [m]   = [%.6f, %.6f, %.6f]\n', r0_I(1), r0_I(2), r0_I(3));
% fprintf('v_ECI [m/s] = [%.6f, %.6f, %.6f]\n', v0_I(1), v0_I(2), v0_I(3));





