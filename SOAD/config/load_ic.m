function ic = load_ic(const, scn)
% LOAD_IC  Compute initial conditions from scenario definition.
%
%   ic = load_ic(const, scn)
%
%   Computes Julian Date, GMST, initial ECI state vectors from Keplerian
%   elements, and sets the initial attitude state.
%
%   Inputs:
%       const - struct from load_constants()
%       scn   - struct from load_scenario()
%
%   Output:
%       ic - struct with fields:
%           .JD_UTC              Julian Date at epoch
%           .gmst0_deg, .gmst0_rad   Initial GMST
%           .q0    [4x1]         Initial quaternion (scalar-first)
%           .w0    [3x1]         Initial angular velocity [rad/s]
%           .r0_I  [3x1]         Initial ECI position [m]
%           .v0_I  [3x1]         Initial ECI velocity [m/s]

    %% Epoch

    ic.JD_UTC = greg2jd(scn.utc0);
    [ic.gmst0_deg, ic.gmst0_rad] = gmst(ic.JD_UTC);

    %% Attitude

    ic.w0 = [0.5; 0.5; 0.5];                                       % [rad/s]

    ic.q0 = [0.989288; 0.094060; 0.078926; 0.094060];              % scalar-first
    ic.q0 = ic.q0 / norm(ic.q0);                                   % enforce unit norm

    %% Orbit (KOE -> ECI)

    [r0, v0] = koe2eci(scn.orb.a, scn.orb.e, scn.orb.i, ...
                        scn.orb.RAAN, scn.orb.w, scn.orb.v0, ...
                        const.earth.mu_km);

    ic.r0_I = r0 * 1e3;                                            % [km] -> [m]
    ic.v0_I = v0 * 1e3;                                            % [km/s] -> [m/s]

end