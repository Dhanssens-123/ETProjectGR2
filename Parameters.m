%%% EXAM4 - GR2 - Parameters
clear variables; close all; clc;

%% Simulation
% Powergui Discrete Time Step
Ts = 5e-5; % ~1/10 of f_switch

%% Reference
% Start Duration
d_start = 0.5;
% Steady Start Duration
d_steady_start = 1;
% Steady Duration
d_steady = 2;
% Steady Stop Duration
d_steady_stop = 1;
% Stop Duration
d_stop = 0.5;
% Total Time
T_tot = 10; %d_start + d_steady_start + d_steady + d_steady_stop + d_stop;

% Steady Speed
N_steady = 1450;

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
% Switching frequency (PWM)
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

%% Control
% TO COMMENT

% Nominal flux
Phi_n = V_r*sqrt(2)/sqrt(3)/(2*pi*f_r); %V/f approximation: we want to keep 
% the flux constant and at the rated value -> Phi_n = V_n/w_n (HYP steady
% state + neglect the voltage drop on R_s) + we divide by sqrt(3) to have
% the phase voltage

% Rated torque (Nm)
T_r = P_r/(N_r*2*pi/60); % T = P/w
% Maximum authorised torque (Nm)
T_sat = 5*T_r; 

Ts_w = 80/f_switch; % 80 choisi un peu au pif honnetement
T_sigma = 1.5*Ts_w; % To check (cf. Slide)
K_sigma = 1;
T_n = 4*T_sigma; % Tuning w/ symmetrical optimum
T_i = 8*K_sigma*T_sigma^2/J; % Tuning w/ symmetrical optimum
Ki = 1/T_i*Ts_w; % *T_sw bc Tustin
Kp = T_n/T_i + Ki/2; %+K_i/2 bc Tustin

%% V/f profile
% TO COMMENT

Zeq = sqrt(R_s^2+(2*pi*f_r*(L_ls+L_m))^2);
Is0 = V_r/Zeq;
V_min = R_s*Is0;
f_min = V_min/(Phi_n*2*pi);

V_f_in =  [-f_r-1 -f_r -f_min 0 f_min f_r f_r+1]*2*pi;
V_f_out = [V_r V_r V_min V_min V_min V_r V_r]/V_r; 

