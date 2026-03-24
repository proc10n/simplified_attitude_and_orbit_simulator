%% main.m
%
%  Loads configuration, computes initial conditions, runs Simulink model.
%

clc; clear;

addpath('config/', 'utils/', 'fsw/');

%% Load configuration

const = load_constants();
scn   = load_scenario(const);
hw    = load_hardware();
fsw   = load_fsw(hw);
ic    = load_ic(const, scn);

%% Temporary 

mekf_cfg = fsw.mekf_cfg; % Need this for now so mekf_compute works
type = scn.type;         % Need this for now so mission_mode works

%% Run simulation

run('model.slx');


