%% main.m
%
% Runs all scripts

clc; clear;

addpath('utils/');

run constants.m
run params.m
run scenario.m
run ic.m

run model.slx


