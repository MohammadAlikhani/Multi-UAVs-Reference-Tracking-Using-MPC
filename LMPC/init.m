% init_drones.m
% Initialize parameters and reference trajectories for visualization
clear all; clc;

% Simulation parameters
g = 9.8;
m = 5;
dt = 0.1; % Sampling period (s)
T_sim = 60; % Simulation time (s)
t = 0:dt:T_sim; % Time vector
N_steps = length(t); % Should be 601

% LMPC parameters
n_drones = 3;
n_x = 3; % [x, y, psi]
d_nominal = 0.4; % Nominal formation distance (m)
