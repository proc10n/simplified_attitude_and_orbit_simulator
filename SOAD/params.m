%% Parameters
% Last Change: 16/03/2026

%% Simulation

sim.Ts = 0.01;

%% Satellite

sat.J = [0.001 0 0;
         0 0.001 0;
         0 0 0.001];

sat.J_inv = inv(sat.J);

sat.rmd = [0.01; 0.01; 0.01];               % [Am^2] Residual magnetic dipole

% --- CubeSat geometry (1U) ---
sat.dim  = 0.1;
sat.d    = sat.dim / 2;
sat.Cd   = 2.2;

d = sat.d;
A = sat.dim^2;

% --- SRP optical properties (uniform for now) ---
sat.cs = 0.1;    % specular reflectivity
sat.cd = 0.1;    % diffuse reflectivity
sat.ca = 0.8;    % absorptivity

%  [--- normal ---]  Area  [--- r_cp ---]       cs      cd      ca
sat.faces = [
     1  0  0    A     d  0  0    sat.cs  sat.cd  sat.ca;
    -1  0  0    A    -d  0  0    sat.cs  sat.cd  sat.ca;
     0  1  0    A     0  d  0    sat.cs  sat.cd  sat.ca;
     0 -1  0    A     0 -d  0    sat.cs  sat.cd  sat.ca;
     0  0  1    A     0  0  d    sat.cs  sat.cd  sat.ca;
     0  0 -1    A     0  0 -d    sat.cs  sat.cd  sat.ca;
];

%% Motor (Portescap 20ECF14 8B 32, 12V variant)

rw.b  = 9.5e-7;                              % Viscous Friction         [Nm·s/rad] (estimated from no-load conditions)
rw.Kt = 13.3e-3;                             % Torque Constant          [Nm/A]
rw.Ke = 13.3e-3;                             % Back-EMF Constant        [V·s/rad]
rw.R  = 4.61;                                % Resistance (phase-phase) [Ohm]
rw.L  = 0.4e-3;                              % Inductance (phase-phase) [H]
rw.Jw = 5e-6;                                % Rotor + RW Inertia       [kg·m^2]

%% Magnetorquer (NCTR-M003, NewSpace Systems)

mtq.m_max    = 0.29;                         % [Am^2] Max dipole moment per axis
mtq.R_coil   = 100;                          % [Ohm]  Coil resistance
mtq.V_supply = 5;                            % [V]    Supply voltage
mtq.m_res    = 0.01;                         % [Am^2] Residual magnetic moment
mtq.linearity = 0.05;                        % [-]    Linearity error (±5%)
mtq.n_rods   = 3;                            % [-]    Number of rods
mtq.area     = 195e-6;                       % [m^2]  Area
mtq.turns    = 1000;                         % [-]    Number of turns
mtq.mass     = 0.030;                        % [kg]   Mass per rod

%% Controller

pd.w_n     = 0.32;                           % Natural Frequency
pd.zeta    = 0.707;                          % Damping Ratio

pd.Kp      = 2 * pd.w_n^2 * sat.J;          % Proportional Gain
pd.Kd      = 2 * pd.w_n * pd.zeta * sat.J;  % Derivative Gain

%% Detumbling

bdot.k     = 4 * pi * 0.001 / 5760;

%% Gyroscope (Analog Devices ADXRS453)
%  Source: ADXRS453 Rev. B datasheet

gyro.sigma_noise = 2.618e-4;                 % [rad/s/sqrt(Hz)] Noise density (0.015 deg/s/sqrt(Hz))
gyro.sigma_arw   = 8.727e-5;                 % [rad/sqrt(s)]    Angle random walk (0.3 deg/sqrt(hr))
gyro.sigma_bias  = 7.716e-5;                 % [rad/s]          Bias instability (16 deg/hr)
gyro.bias0       = [0; 0; 0];                % [rad/s]          Initial bias (unknown; estimated by MEKF)
gyro.range       = 300 * pi/180;             % [rad/s]          Max measurable rate (±300 deg/s)
gyro.resolution  = 16;                       % [bits]           ADC resolution
gyro.fs          = 100;                      % [Hz]             Sample rate
gyro.dt          = 1/gyro.fs;                % [s]              Sample period

%% Magnetometer (PNI RM3100)
%  Source: RM3100 datasheet, Regoli et al. (2018)

mag.sigma_noise = 15e-9;                     % [T]     Noise per axis (1-sigma, CC=200)
mag.bias        = [0; 0; 0];                 % [T]     Sensor bias (dominated by s/c residual field)
mag.range       = 1100e-6;                   % [T]     Measurement range (±1100 uT)
mag.cc          = 200;                       % [-]     Cycle count setting
mag.fs          = 50;                        % [Hz]    Sample rate at CC=200 (single-axis ~75 Hz, 3-axis ~37 Hz)
mag.dt          = 1/mag.fs;                  % [s]     Sample period

%% Sun Sensor (NewSpace Systems AQUILA-D02 / NFSS-411)
%  Source: NSS Sun Sensor Datasheet (2024)
%  Notes: Fine sun sensor, slit-type (sapphire window).
%         Outputs digital sun vector via RS-485.
%         Accuracy spec is over the full 140 deg FOV.
%         Eclipse model disables measurement when satellite is in shadow.

ss.sigma_angle = 0.05 * pi/180;              % [rad]   Angle noise (1-sigma, 0.2 deg RMS)
ss.fov_half    = 70 * pi/180;               % [rad]   Half-angle FOV (70 deg -> 140 deg total)
ss.bias        = [0; 0; 0];                 % [-]     Unit vector bias (small; albedo-immune design)
ss.fs          = 5;                          % [Hz]    Update rate
ss.dt          = 1/ss.fs;                    % [s]     Sample period

%% Star Tracker (arcsec Sagitta)
%  Source: Sagitta Star Tracker Datasheet v2
%  Notes: Outputs quaternion (inertial-to-body).
%         Noise is anisotropic: much better cross-boresight than around boresight.
%         Model: q_meas = q_true ⊗ dq(sigma_cross, sigma_cross, sigma_bore)
%         Sun exclusion angle: 40 deg (with integrated baffle).

st.sigma_cross = 2 / 3600 * pi/180;         % [rad]   Cross-boresight noise (1-sigma, 2 arcsec)
st.sigma_bore  = 10 / 3600 * pi/180;        % [rad]   Around-boresight noise (1-sigma, 10 arcsec)
st.sun_excl    = 40 * pi/180;               % [rad]   Sun exclusion angle (baffle)
st.earth_excl  = 25 * pi/180;               % [rad]   Earth limb exclusion (typical, not in datasheet)
st.boresight   = [0; 0; 1];                 % [-]     Boresight direction in body frame (default: +Z)
st.fs          = 10;                         % [Hz]    Update rate
st.dt          = 1/st.fs;                    % [s]     Sample period
st.availability = 0.99;                      % [-]     Lost-in-space sky coverage (>99%)

%% MEKF Configuration

mekf_cfg.sigma_gyro    = gyro.sigma_arw;          % [rad/sqrt(s)]  Attitude process noise (ARW)
mekf_cfg.sigma_bias_rw = gyro.sigma_bias;          % [rad/s]        Bias random walk
mekf_cfg.sigma_mag     = mag.sigma_noise;           % [T]            Magnetometer measurement noise
mekf_cfg.sigma_sun     = ss.sigma_angle;            % [rad]          Sun sensor measurement noise
mekf_cfg.sigma_st      = st.sigma_cross;            % [rad]          Star tracker noise (isotropic approx)
mekf_cfg.P0_att        = 1e-2;                      % [rad^2]        Initial attitude covariance (per axis)
mekf_cfg.P0_bias       = 1e-4;                      % [(rad/s)^2]    Initial bias covariance (per axis)
mekf_cfg.B0            = gyro.bias0;                 % [rad/s]        Initial bias estimate
mekf_cfg.N_st          = max(1, round(st.dt / sim.Ts));  % [-]       ST update decimation (steps between updates)