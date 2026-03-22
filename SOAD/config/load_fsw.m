function fsw = load_fsw(hw)
% LOAD_FSW  Flight software configuration.
%
%   fsw = load_fsw(hw)
%
%   Defines simulation settings, controller gains, and estimation
%   algorithm configuration. Separated from hardware because these
%   are tuning knobs, not datasheet values.
%
%   Inputs:
%       hw - struct from load_hardware()
%
%   Output:
%       fsw - struct with fields:
%           .sim       Simulation settings
%           .pd        PD controller gains
%           .bdot      B-dot detumbling gain
%           .mekf_cfg  MEKF filter configuration

    %% Simulation

    fsw.sim.Ts = 0.01;                          % [s] Simulation timestep
    fsw.sim.Tenv = 1;                           % [s] Environment timestep

    %% PD Controller

    fsw.pd.w_n  = 0.64;                         % [rad/s] Natural frequency
    fsw.pd.zeta = 0.707;                         % [-]     Damping ratio

    fsw.pd.Kp = 2 * fsw.pd.w_n^2 * hw.sat.J;              % Proportional gain
    fsw.pd.Kd = 2 * fsw.pd.w_n * fsw.pd.zeta * hw.sat.J;  % Derivative gain

    %% B-dot Detumbling

    fsw.bdot.k = 4 * pi * 0.001 / 5760;

    %% MEKF Configuration

    fsw.mekf_cfg.sigma_gyro    = hw.gyro.sigma_arw;    % [rad/sqrt(s)]  Attitude process noise (ARW)
    fsw.mekf_cfg.sigma_bias_rw = hw.gyro.sigma_bias;   % [rad/s]        Bias random walk
    fsw.mekf_cfg.sigma_mag     = hw.mag.sigma_noise;   % [T]            Magnetometer measurement noise
    fsw.mekf_cfg.sigma_sun     = hw.ss.sigma_angle;    % [rad]          Sun sensor measurement noise
    fsw.mekf_cfg.sigma_st      = hw.st.sigma_cross;    % [rad]          Star tracker noise (isotropic approx)
    fsw.mekf_cfg.P0_att        = 1e-2;                 % [rad^2]        Initial attitude covariance (per axis)
    fsw.mekf_cfg.P0_bias       = 1e-4;                 % [(rad/s)^2]    Initial bias covariance (per axis)
    fsw.mekf_cfg.B0            = hw.gyro.bias0;        % [rad/s]        Initial bias estimate
    fsw.mekf_cfg.N_st          = max(1, round(hw.st.dt / fsw.sim.Ts));  % [-] ST update decimation

end