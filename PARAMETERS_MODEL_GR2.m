%%% EXAM4 - GR2 - Parameters
clear variables; close all; clc;

%% Simulation
%%% Switching Time

% Frequency index (mf = fsw/f ; Always odd)
f = 50;
m_f = 301; % 15 050 Hz
% Switching frequency (PWM)
f_switch = f*m_f; % Between 10kHz and 20kHz
% Switching time
T_switch = 1/f_switch;

%%% Speed Control Sampling Time

% Sampling time of the speed controller
fs_w = 1/10*f_switch; % 10 to 100 times slower than the switching (PWM) BW
% Speed Control Sampling Time
Ts_w = 1/fs_w;

%%% Powergui Discrete Step Time
Ts = 1e-5; % Ts <= T_switch & Ts == Even

%% Induction Motor

% Rated power (W)
P_r = 19e3; 
% Rated line voltage (V)
V_r = 400;
% Stator voltage
V_s = V_r/sqrt(3); % Phase voltage
% Pole Number
P = 4;
% Pole pairs
p = P/2;
% Rated frenquecy (Hz)
f_r = 50;
w_s = 2*pi*f_r;
w_s_mech = w_s/p; % Mechanical frequency
% Stator resistance (Ohm)
R_s = 0.0629;
% Rotor resistance (Ohm)
R_r = 0.1091;
% Mutual inductance (H)
L_m = 0.0408;
X_m = w_s*L_m;
R_fe = 213.2; % (Ohm)
% Stator leakage inductance (H)
L_ls = 1.8e-3;
X_ls = w_s*L_ls;
% Rotor leakage inductance (H)
L_lr = 1.8e-3;
X_lr = w_s*L_lr;
% Rated speed (rpm)
N_r = 1460;
% Moment of inertia (kg.m^2)
J = 0.2799;
% Friction coef (N.m.sec/r)
B = 0.0129;

% Dumping frequency 
Xi = 0.85;

% Rated Synchronous Speed
N_synchro = 60*f_r/p;
% Rated Slip
slip_r = (N_synchro - N_r)/N_synchro;

% Z_rotor
Z_r = R_r/slip_r + 1j*X_lr;
% Z_magnetisation
Z_m = (R_fe*1j*X_m)/(R_fe + 1j*X_m);
% Z_stator
Z_s = R_s + 1j*X_ls;
% Z_equivalent
Z_eq = Z_s + (Z_r*Z_m)/(Z_r+Z_m);

% Currents
I_s = V_s/abs(Z_eq);
I_r = I_s*abs(Z_m/(Z_r+Z_m));

% Torques (3 phases -> Multiply by 3/2)
T_mech = 3/2*1/w_s_mech*V_s^2/((R_s+R_r/slip_r)^2 + (X_ls+X_lr)^2)*R_r/slip_r;
T_max = 3/2*1/(2*w_s_mech)*V_s^2/(R_s + sqrt(R_s^2+(X_ls+X_lr)^2)); % Breakdown

% Rated torque (Nm)
T_r = P_r/(N_r*2*pi/60); % T = P/w

% Nominal flux
Phi_n = V_s/w_s; % Keep it constat (V/f control)

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
R_on_rect = 1e-3; % TO CHECK
% Diode Vf (V)
V_f_rect = 0.8; % TO CHECK
% Diode Snubber R (Ohm)
R_snub_rect = 500; % TO CHECK (cf. Matlab Documentation)
% Diode Subber C (F)
C_snub_rect = 250e-9; % TO CHECK (cf. Matlab Documentation)
 
%% Inverter

% Input DC-link capacitor (F)
C_dc = 15e-3;

% FET Ron (Ohm)
% R_on_inv = 1e-1; % TO CHECK
R_on_inv = 4e-3; % TO CHECK
% Internal Diode Resistance (Ohm)
R_d_inv  = 4.17e-3; % TO CHECK
% Internal diode inductance (H)
L_on_inv = 0; % TO CHECK
% Internal Diode Vf (V)
V_f_inv = 0; % TO CHECK
% Snubber R (Ohm)
R_snub_inv = 1e5; % TO CHECK (cf. Matlab Documentation)
% Snubber C (F)
C_snub_inv = inf; % TO CHECK (cf. Matlab Documentation)

%% Control - Reference

% Total Duration
T_tot = 10;

% Speed Reference Steps
t_rampup_start = 0.05*T_tot;
t_rampup_end = 0.4*T_tot;
t_rampdown_start = 0.6*T_tot;
t_rampdown_end = 0.95*T_tot;
% Torque Load Delay
t_delay_up = 0.05*T_tot;
t_delay_down = 0.01*T_tot;

% Steady Speed (RPM)
N_steady = N_r;

% Ramp
Max_Slope_Rads = T_r/J;
Max_Slope_RPM = 30/pi*Max_Slope_Rads;
%
Slope_up = N_steady/(t_rampup_end - t_rampup_start);
Slope_down = -N_steady/(t_rampdown_end - t_rampdown_start);

% Torque load (Nm)
T_load = 0.8*T_r; % 80% of Rated torque

% Maximum authorised torque (Nm)
T_sat = 0.9*T_max; % Less than Breakdown torque

%% Control - Controller Design

% Actuator - Equivalent Delay
T_sigma = 1.5*Ts_w; % Equivalent time cst (delay in a controlled system), 
K_sigma = 1; % Equivalent gain

% Controller - Magnitude Optimum Method
T_n = J/B; 
T_i = 2*K_sigma*1/B*T_sigma; 
%
Kp = T_n/T_i; % OR J*fs_w
Ki = 1/T_i; % OR B*fs_w

% Discrete Control - Tustin Approximation
Kp = Kp + Ki*Ts_w/2;
Ki = Ki*Ts_w;

%% V/f profile

% Resistive drop compensation for small frequencies
Zeq = abs(Z_s+Z_m); % At no load
Is0 = V_s/abs(Zeq);

V_min = R_s*Is0;
f_min = V_min/(Phi_n*2*pi);

V_f_in =  [-f_r-1 -f_r -f_min 0 f_min f_r f_r+1]*2*pi;
V_f_out = [V_s V_s V_min V_min V_min V_s V_s]/V_s;