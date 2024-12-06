%%% EXAM4 - GR2 - Parameters
clear variables; close all; clc;

%% Induction Motor

% Rated power (W)
P_r = 19e3; 
% Rated line voltage (V)
V_r = 400;
% Rated frenquecy (Hz)
f_r = 50;
% Stator resistance (Ohm)
R_s = 0.0629;
% Rotor resistance (Ohm)
R_r = 0.1091;
% Mutual inductance (H)
L_m = 0.0408;
R_fe = 213.2; % (Ohm)
% Stator leakage inductance (mH)
L_ls = 1.8;
% Rotor leakgae inductance (mH)
L_lr = 1.8;
% Rated speed (rpm)
N_r = 1460;
% Moment of inertia (kg.m^2)
J = 0.2799;
% Friction coef (N.m.sec/r)
B = 0.0129;
% Pole Number
P = 4;
% Dumping frequency 
E = 0.85;

%% Power Supply

% Phase-to-phase RMS voltage (V)
U_ac = 380;
f_grid = 50;
R_grid = 1e-3; % TO CHECK
L_grid = 1e-12; % TO CHECK

%% Rectifier

% Output inductor (H)
L_rectif = 10e-3;
% Diode Ron (Ohm)
R_on = 1e-3;
% Diode Vf (V)
V_f = 0;
% Diode Snubber R (Ohm)
R_snubber = 500;
% Diode Subber C (F)
C_snubber = 250e-9;

%% Inverter

% Input DC-link capacitor (F)
C_dc = 15e-3;
% FET Ron (Ohm)
R_on_fet = 1e-1;
% Internal Diode Resistance (Ohm)
R_d = 1e-2;
% Internal Diode Vf (V)
V_f_inv = 0;
% Snubber R (Ohm)
R_snub_inv = 1e5;
% Snubber C (F)
C_snub_inv = inf;

