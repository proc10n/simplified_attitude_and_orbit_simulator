%% main.m
%
% Runs all scripts

clc; clear;

addpath('utils/');
addpath('fsw/');

run constants.m
run scenario.m
run params.m
run ic.m

run model.slx


