%% export_vts.m
%
%  Exports Simulink position and attitude data to CIC OEM / AEM files
%  compatible with VTS (CNES Visualisation Tool for Space).
%
%  Assumes workspace contains:
%    out.tout    - simulation time vector [s]
%    out.pos_eci - ECI position [m], Nx3 or 3xN
%    out.q_real  - attitude quaternion (scalar-first), Nx4 or 4xN
%    ic.JD_UTC   - Julian Date at epoch (from load_ic)
%
%  Run this after the Simulink simulation completes.
%
%
% Made with Claude <3

%% ---- User settings ----

dt_out   = fsw.sim.Ts;                          % [s] output interval (match VTS needs)
vts_dir  = '~/Desktop/VTS/Vts-Linux-64bits-3.9.1/Data/CubeSat/Data/';
oem_file = fullfile(vts_dir, 'cubesat_orbit.txt');
aem_file = fullfile(vts_dir, 'cubesat_attitude.txt');

obj_name = 'CubeSat';
obj_id   = 'CubeSat';

%% ---- Extract and reshape data ----

% Handle both plain arrays and timeseries objects
if isstruct(out.pos_eci) || isa(out.pos_eci, 'timeseries')
    pos  = squeeze(out.pos_eci.Data);
else
    pos  = out.pos_eci;
end

if isstruct(out.q_real) || isa(out.q_real, 'timeseries')
    quat = squeeze(out.q_real.Data);
else
    quat = out.q_real;
end

t = out.tout(:);                        % column vector [s]

% Ensure Nx3 / Nx4
if size(pos,1) == 3 && size(pos,2) ~= 3
    pos = pos.';
end
if size(quat,1) == 4 && size(quat,2) ~= 4
    quat = quat.';
end

%% ---- Downsample to dt_out ----

t_out = (0 : dt_out : t(end)).';
pos_out  = interp1(t, pos,  t_out, 'linear');
quat_out = interp1(t, quat, t_out, 'linear');

% Re-normalise quaternions after interpolation
quat_out = quat_out ./ vecnorm(quat_out, 2, 2);

%% ---- Time conversion ----
%  VTS CIC format: MJD_day  seconds_of_day

JD0  = ic.JD_UTC;                       % epoch Julian Date
MJD0 = JD0 - 2400000.5;                 % epoch MJD

% For each sample: MJD day number and seconds within that day
mjd_continuous = MJD0 + t_out / 86400;
mjd_day = floor(mjd_continuous);
sec_day = (mjd_continuous - mjd_day) * 86400;

% Clean up floating-point residuals
sec_day = round(sec_day, 6);

%% ---- Position: m -> km ----

pos_km = pos_out * 1e-3;

%% ---- Write OEM ----

now_str = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));

fid = fopen(oem_file, 'w');

fprintf(fid, 'CIC_OEM_VERS = 2.0\n');
fprintf(fid, 'CREATION_DATE  = %s\n', now_str);
fprintf(fid, 'ORIGINATOR     = SOAD\n');
fprintf(fid, 'META_START\n');
fprintf(fid, 'OBJECT_NAME = %s\n', obj_name);
fprintf(fid, 'OBJECT_ID = %s\n', obj_id);
fprintf(fid, 'CENTER_NAME = EARTH\n');
fprintf(fid, 'REF_FRAME   = EME2000\n');
fprintf(fid, 'TIME_SYSTEM = UTC\n');
fprintf(fid, 'META_STOP\n');

for k = 1:length(t_out)
    fprintf(fid, '%d %g %f %f %f\n', ...
        mjd_day(k), sec_day(k), ...
        pos_km(k,1), pos_km(k,2), pos_km(k,3));
end

fclose(fid);

%% ---- Write AEM ----

fid = fopen(aem_file, 'w');

fprintf(fid, 'CIC_AEM_VERS = 1.0\n');
fprintf(fid, 'CREATION_DATE  = %s\n', now_str);
fprintf(fid, 'ORIGINATOR     = SOAD\n');
fprintf(fid, 'META_START\n');
fprintf(fid, 'OBJECT_NAME = %s\n', obj_name);
fprintf(fid, 'OBJECT_ID = %s\n', obj_id);
fprintf(fid, 'REF_FRAME_A = EME2000\n');
fprintf(fid, 'REF_FRAME_B = SC_BODY_1\n');
fprintf(fid, 'ATTITUDE_DIR = A2B\n');
fprintf(fid, 'TIME_SYSTEM = UTC\n');
fprintf(fid, 'ATTITUDE_TYPE = QUATERNION\n');
fprintf(fid, 'META_STOP\n');

for k = 1:length(t_out)
    fprintf(fid, '%d %g %f %f %f %f\n', ...
        mjd_day(k), sec_day(k), ...
        quat_out(k,1), quat_out(k,2), quat_out(k,3), quat_out(k,4));
end

fclose(fid);

%% ---- Done ----

fprintf('OEM written to: %s  (%d points)\n', oem_file, length(t_out));
fprintf('AEM written to: %s  (%d points)\n', aem_file, length(t_out));