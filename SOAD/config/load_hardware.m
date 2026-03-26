function hw = load_hardware()
% LOAD_HARDWARE  Spacecraft hardware parameters.
%
%   hw = load_hardware()
%
%   Defines satellite body properties, sensor models, and actuator models.
%   All values are datasheet-sourced or derived from hardware specs.
%
%   Output:
%       hw - struct with fields:
%           .sat   Satellite body (inertia, geometry, optical properties)
%           .rw    Reaction wheel / DC motor (Portescap 20ECF14)
%           .mtq   Magnetorquer (NSS NCTR-M003)
%           .gyro  Gyroscope (ADXRS453)
%           .mag   Magnetometer (PNI RM3100)
%           .ss    Sun sensor (NSS AQUILA-D02)
%           .st    Star tracker (arcsec Sagitta)

    %% Satellite body

    hw.sat.J     = diag([0.001, 0.001, 0.001]); % [kg*m^2] Inertia tensor
    hw.sat.J_inv = inv(hw.sat.J);

    hw.sat.rmd   = [0.01; 0.01; 0.01];          % [Am^2] Residual magnetic dipole

    % CubeSat geometry (1U)
    hw.sat.dim = 0.1;                            % [m]   Side length
    hw.sat.d   = hw.sat.dim / 2;                 % [m]   Half-side (CoP offset)
    hw.sat.Cd  = 2.2;                            % [-]   Drag coefficient
    hw.sat.r_cm = [0.005; 0.003; -0.002];        % [m]   CoM offset from geometric center

    % SRP optical properties
    hw.sat.cs = 0.1;                             % [-] Specular reflectivity
    hw.sat.cd = 0.1;                             % [-] Diffuse reflectivity
    hw.sat.ca = 0.8;                             % [-] Absorptivity

    % Face table: [normal(3), area, r_cp(3), cs, cd, ca]
    dim = hw.sat.dim;
    d   = hw.sat.d;
    fA  = dim^2;
    r_cm = hw.sat.r_cm;

    hw.sat.faces = [
         1  0  0   fA    d-r_cm(1)   -r_cm(2)    -r_cm(3)    hw.sat.cs  hw.sat.cd  hw.sat.ca;
        -1  0  0   fA   -d-r_cm(1)   -r_cm(2)    -r_cm(3)    hw.sat.cs  hw.sat.cd  hw.sat.ca;
         0  1  0   fA     -r_cm(1)   d-r_cm(2)   -r_cm(3)    hw.sat.cs  hw.sat.cd  hw.sat.ca;
         0 -1  0   fA     -r_cm(1)  -d-r_cm(2)   -r_cm(3)    hw.sat.cs  hw.sat.cd  hw.sat.ca;
         0  0  1   fA     -r_cm(1)   -r_cm(2)    d-r_cm(3)   hw.sat.cs  hw.sat.cd  hw.sat.ca;
         0  0 -1   fA     -r_cm(1)   -r_cm(2)   -d-r_cm(3)   hw.sat.cs  hw.sat.cd  hw.sat.ca;
    ];

    %% DC Motor (Portescap 20ECF14 8B 32, 12V variant)

    hw.rw.b  = 9.5e-7;                          % [Nm*s/rad] Viscous friction (estimated)
    hw.rw.Kt = 13.3e-3;                         % [Nm/A]     Torque constant
    hw.rw.Ke = 13.3e-3;                         % [V*s/rad]  Back-EMF constant
    hw.rw.R  = 4.61;                            % [Ohm]      Resistance (phase-phase)
    hw.rw.L  = 0.4e-3;                          % [H]        Inductance (phase-phase)
    hw.rw.Jw = 5e-6;                            % [kg*m^2]   Rotor + RW inertia

    %% Magnetorquer (NSS NCTR-M003)

    hw.mtq.m_max     = 0.29;                    % [Am^2] Max dipole moment per axis
    hw.mtq.R_coil    = 100;                      % [Ohm]  Coil resistance
    hw.mtq.V_supply  = 5;                        % [V]    Supply voltage
    hw.mtq.m_res     = 0.01;                     % [Am^2] Residual magnetic moment
    hw.mtq.linearity = 0.05;                     % [-]    Linearity error (±5%)
    hw.mtq.n_rods    = 3;                        % [-]    Number of rods
    hw.mtq.area      = 195e-6;                   % [m^2]  Area
    hw.mtq.turns     = 1000;                     % [-]    Number of turns
    hw.mtq.mass      = 0.030;                    % [kg]   Mass per rod

    %% Gyroscope (Analog Devices ADXRS453)
    %  Source: ADXRS453 Rev. B datasheet

    hw.gyro.sigma_noise = 2.618e-4;             % [rad/s/sqrt(Hz)] Noise density (0.015 deg/s/sqrt(Hz))
    hw.gyro.sigma_arw   = 8.727e-5;             % [rad/sqrt(s)]    Angle random walk (0.3 deg/sqrt(hr))
    hw.gyro.sigma_bias  = 7.716e-5;             % [rad/s]          Bias instability (16 deg/hr)
    hw.gyro.bias0       = [0; 0; 0];            % [rad/s]          Initial bias (unknown; estimated by MEKF)
    hw.gyro.range       = 300 * pi/180;          % [rad/s]          Max measurable rate (±300 deg/s)
    hw.gyro.resolution  = 16;                    % [bits]           ADC resolution
    hw.gyro.fs          = 100;                   % [Hz]             Sample rate
    hw.gyro.dt          = 1 / hw.gyro.fs;        % [s]              Sample period

    %% Magnetometer (PNI RM3100)
    %  Source: RM3100 datasheet, Regoli et al. (2018)

    hw.mag.sigma_noise = 15e-9;                  % [T]   Noise per axis (1-sigma, CC=200)
    hw.mag.bias        = [0; 0; 0];              % [T]   Sensor bias
    hw.mag.range       = 1100e-6;                % [T]   Measurement range (±1100 uT)
    hw.mag.cc          = 200;                    % [-]   Cycle count setting
    hw.mag.fs          = 1/0.05;                     % [Hz]  Sample rate at CC=200
    hw.mag.dt          = 1 / hw.mag.fs;          % [s]   Sample period

    %% Sun Sensor (NSS AQUILA-D02 / NFSS-411)
    %  Source: NSS Sun Sensor Datasheet (2024)

    hw.ss.sigma_angle = 0.05 * pi/180;          % [rad] Angle noise (1-sigma, 0.2 deg RMS)
    hw.ss.fov_half    = 70 * pi/180;             % [rad] Half-angle FOV (140 deg total)
    hw.ss.bias        = [0; 0; 0];               % [-]   Unit vector bias
    hw.ss.fs          = 5;                        % [Hz]  Update rate
    hw.ss.dt          = 1 / hw.ss.fs;             % [s]   Sample period

    %% Star Tracker (arcsec Sagitta)
    %  Source: Sagitta Star Tracker Datasheet v2

    hw.st.sigma_cross  = 2 / 3600 * pi/180;      % [rad] Cross-boresight noise (1-sigma, 2 arcsec)
    hw.st.sigma_bore   = 10 / 3600 * pi/180;     % [rad] Around-boresight noise (1-sigma, 10 arcsec)
    hw.st.sun_excl     = 40 * pi/180;            % [rad] Sun exclusion angle (baffle)
    hw.st.earth_excl   = 25 * pi/180;            % [rad] Earth limb exclusion (typical)
    hw.st.boresight    = [0; 0; 1];              % [-]   Boresight direction in body frame
    hw.st.fs           = 10;                     % [Hz]  Update rate
    hw.st.dt           = 1 / hw.st.fs;           % [s]   Sample period
    hw.st.availability = 0.99;                   % [-]   Lost-in-space sky coverage

end