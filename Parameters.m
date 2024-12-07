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
% Stator leakage inductance (H)
L_ls = 1.8e-3;
% Rotor leakage inductance (H)
L_lr = 1.8e-3;
% Rated speed (rpm)
N_r = 1460;
% Moment of inertia (kg.m^2)
J = 0.2799;
% Friction coef (N.m.sec/r)
B = 0.0129;
% Pole Number
P = 4;
% Dumping frequency 
Xi = 0.85;

%% Power Supply

% Phase-to-phase RMS voltage (V)
U_ac = 380;
f_grid = 50;
R_grid = 0.2; % TO CHECK
L_grid = 20e-6; % TO CHECK

%% Rectifier

% Output inductor (H)
L_rectif = 10e-3;

% Diode Ron (Ohm)
R_on_rect = 1e-3;
% Diode Vf (V)
V_f_rect = 0;
% Diode Snubber R (Ohm)
R_snub_rect = 500;
% Diode Subber C (F)
C_snub_rect = 250e-9;

%% Inverter

% Frequency index (mf = fsw/f)
m_f = 39;
% Amplitude index (m)
m = 1;
% Switching frequency
f_switch = f_r*m_f;

% Input DC-link capacitor (F)
C_dc = 15e-3;

% FET Ron (Ohm)
R_on_inv = 1e-1;
% Internal Diode Resistance (Ohm)
R_d_inv  = 1e-2;
% Internal diode inductance (H)
L_on_inv = 0;
% Internal Diode Vf (V)
V_f_inv = 0;
% Snubber R (Ohm)
R_snub_inv = 1e5;
% Snubber C (F)
C_snub_inv = inf;

