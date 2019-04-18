%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% heli.m
%% Matlab script to be run before Simulink files
%% ACS336 / ACS6336 / ACS6110
%% Last revised: 23.02.2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;      % close all figures
clear all;      % clear workspace variables

%% Define Discrete Time MyDAQ Dynamics
T           = 0.015;            % Sample period (s)
ADC_Vres    = 20/((2^16)-1);    % ADC voltage resolution (V/bit)
Encoder_res = 2*pi/500;         % Encoder resolution (rad/wheel count)
DAC_Vres    = 20/((2^16)-1);    % DAC voltage resolution (V/bit)
DAC_lim_u   = 10;               % DAC upper saturation limit (V)
DAC_lim_l   = 0;                % DAC enforced lower saturation limit (V)
%% Define Continuous Time Helicopter Dynamics
g   = 9.81;     % Gravitational acceleration (ms^-2) 
% Rigid body parameters
% Masses and lengths
m1  = 0.0505;   % mass of fan assembly (kg)
m2  = 0.100;    % mass of counterweight (kg)
l1  = 0.110;    % distance from helicopter arm to elevation axis (m);
l2  = 0.070;    % distance from fan centres to pitch axis (m);
l3  = 0.108;    % distance from counterweight to elevation axis (m);
% Inertias
Je  = 2*m1*(l1^2)+m2*(l3^2);    % Inertia about elevation axis (kg*m^2);
Jt  = Je;                       % Travel axis inertia
Jp  = 2*m1*(l2^2);              % Pitch axis inertia
% Constraints
p_lim_u     = 80*pi/180;    % Upper pitch axis limit (rad)
p_lim_l     = -80*pi/180;   % Lower pitch axis limit (rad)
e_lim_u     = 50*pi/180;    % Upper elevation axis limit (rad)
e_lim_l     = -50*pi/180;   % Lower elevation axis limit (rad)

%% Ex 1: DETERMINE PITCH AXIS SPRING AND DAMPING COEFFICIENTS %%%%%%%%%%%%%
% Pitch axis spring and damping constants
k_s = .0015;  %.0015         % Spring constant (kg*m^2*s^-2)
k_d = .0005;  %0.0005         % Viscous damping (kg*m^2*s^-1)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%% Ex 2: DETERMINE POWER AMPLIFIER GAIN AND SATURATION LIMITS %%%%%%%%%%%%%
% Power amplifier
k_a         = 1.2;   % Power amplifier voltage gain
amp_sat_u   = 12; %5.7  %12 % Power amplifier upper saturation limit (V)
amp_sat_l   = 0;   % Power amplifier lower saturation limit (V)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Ex 3: CONSTRUCT FAN MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fan voltage - thrust steady state behaviour
SteadyStateFan = load('C:\Users\Brian\Desktop\sheffield\quarter 2\Acs6336 Hardware Rapid control\lab exercises\3\FanModule\steadystate.txt');
V_ab = SteadyStateFan(:,1)*k_a;  % Fan voltage input (V)
Fss_ab= SteadyStateFan(:,2); % Steady-state fan thrust output (N)         

% Fan voltage - thrust transient model.
tau = .65;             % 1st order time constant

%% Ex 4: DETERMINE EQUILIBRIUM CONTROL SIGNAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant control input required to achieve hover
%U_e = 5.8;
U_e = 6.5;          %5.8 6.0 good rise 7.5 max = 8.0 % Voltage output from myDAQ

%% Ex 5: DEFINE LTI STATE-SPACE CONTROL MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Approximate the fan voltage/thrust relationship by an affine           %
%  function of the form F_ab = alpha*V_ab+beta. Define alpha and beta.    %
alpha = 0.0031; % 0.004
beta =  -0.009; % -0.01
F_ab = alpha*V_ab+beta;
%plot(V_ab,F_ab,'k');

% grid on; hold on;
% xlabel('Fan Voltage (V)');
% ylabel('Output Thrust (N)');           
% plot(V_ab, Fss_ab, 'bx') % plot raw thrust data
% plot(V_ab,alpha*V_ab+beta,'r-'); % plot linear approximation

%  State vector x:=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot]     %
%  Note; these states model the dynamics of small perturbations around    %
%  the state of steady, level hover.                                      %
%  Define the control model given by x_dot = Ax + Bu, y = Cx + Du         %
A = [0 0 0 1 0 0; 
    0 0 0 0 1 0; 
    0 0 0 0 0 1; 
    0 0 0 0 0 0; 
    0 -k_s/Jp 0 0 -k_d/Jp 0; 
    0 -2*l1*(alpha*k_a*U_e+beta)/Jt 0 0 0 0];

B = [0 0; 0 0; 
    0 0; 
    l1*alpha*k_a/Je l1*alpha*k_a/Je; 
    l2*alpha*k_a/Jp -l2*alpha*k_a/Jp;
    0 0]; 

C = [1 0 0 0 0 0; 
    0 1 0 0 0 0; 
    0 0 1 0 0 0];

D = [0 0; 
    0 0; 
    0 0];

%% Ex 6: Discrete Time Full state feedback control %%%%%%%%%%%%%%%%%%%%%%%%
% State feedback control design (integral control via state augmentation)
% x:=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot; int_elev; int_trav]
% Define augmented system matrices
Cr      = [1 0 0 0 0 0
           0 0 1 0 0 0];    % Elevation and travel are controlled outputs
r       = 2;                                % number of reference inputs
n       = size(A,2);                        % number of states
q       = size(Cr,1);                       % number of controlled outputs
Dr      = zeros(q,2);
Aaug    = [A zeros(n,r); -Cr zeros(q,r)];
Baug    = [B; -Dr];
% Define LQR weighting matrices (Linear–quadratic regulator)

% Qx = eye(8);    % State penalty (8x8)8.0, 8.0, 6.25 7.0
% Qx(1,1) = 1;    %elev 200 300 maybe 1 1000 b 1 1 (distance)
% Qx(2,2) = 15;    %pitch 150 150 maybe 50 50 b 15 15
% Qx(3,3) = 50;    %trav 50 15 maybe 20 20 b 55 60
% Qx(4,4) = 300;    %elev_dot 200 1000 maybe 850 1000 b 850 850 (lower, faster)
% Qx(5,5) = 300;    %pitch_dot 50 200 maybe 200 200 b 365 365
% Qx(6,6) = 350;    %trav_dot 50 15 maybe 20 20 b 280 300
% Qx(7,7) = 15;    %elev_integral b 15 5
% Qx(8,8) = 16;    %trav_intergral b 20 22

Qx = diag([1,15,50,550,300,280,10,14]);
Qx = diag([1,15,50,200,300,300,17,14]);

%the perfect looking one ue = 6.0, [1,15,55,860,600,280,5,22]
% ue = 5.8, Qx = diag([1,15,50,550,300,280,10,14]);

%works pretty well ue = 6.8, [1,15,55,1000,600,280,12,22] <30sec
% ue = 6.8, [1,15,100,800,600,400,35,22] 
% ue = 6.8, [1,15,55,850,600,300,18,15] 
% ue = 6.8, [1,15,55,550,600,300,18,15] good
% ue = 6.8, Qx = diag([1,15,55,600,600,350,19,16]) 
% ue = 6.8, Qx = diag([1,15,50,300,300,350,15,16]); 
% ue = 6.8, Qx = diag([1,15,50,175,200,300,17,14]); the one
%Qx = diag([1,15,50,200,300,300,17,14]);
%ue = 6.5, Qx = diag([1,15,50,200,300,300,19,14]);
% ue = 6.5, Qx = diag([1,15,50,220,300,300,19,14]);

Qu = diag([1,1]);    % Control penalty (2x2)


% Discrete-Time LQR synthesis
Kdtaug  = lqrd(Aaug,Baug,Qx,Qu,T);      % DT state-feedback controller
Kdt     = Kdtaug(:,1:n); Kidt = -Kdtaug(:,n+1:end);
% Discrete-Time Kalman Filter Design
sysdt = c2d(ss(A,B,C,D),T,'zoh');     % Generate discrete-time system
Adt   = sysdt.a; Bdt = sysdt.b; Cdt = sysdt.c; Ddt = sysdt.d;
%  Kalman filter design; x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v
Gdt     = 1e-1*eye(n);
Hdt     = zeros(size(C,1),size(Gdt,2)); % No process noise on measurements
%Rw      = diag([Qx(1,1),Qx(2,2),Qx(3,3),Qx(4,4),Qx(5,5),Qx(6,6)]);   % Process noise covariance matrix (6x6)
Rw      = eye(6);

%Rv      = eye(3);
Rv      = eye(3) * 0.001;   % Measurement noise covariance matrix (3x3)
sys4kf  = ss(Adt,[Bdt Gdt],Cdt,[Ddt Hdt],T);
[kdfilt Ldt] = kalman(sys4kf,Rw,Rv);     % Kalman filter synthesis

%% Output Files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Uncomment these lines when ready to implement your feedback controller  %
k_amp       = k_a;
Vfan        = V_ab;
Thrust      = Fss_ab;

KF_Initial  = [e_lim_l 0 0 0 0 0];
Kdi_Initial = [0 0];

csvwrite('aMatrix.txt',kdfilt.a)
csvwrite('bMatrix.txt',kdfilt.b)
csvwrite('cMatrix.txt',kdfilt.c(size(C,1)+1:end,:))
csvwrite('dMatrix.txt',kdfilt.d(size(C,1)+1:end,:))
csvwrite('crMatrix.txt',Cr)
csvwrite('KdMatrix.txt',Kdt)
csvwrite('KdiMatrix.txt',Kidt)
csvwrite('FanChar.txt',[Vfan,Thrust])
csvwrite('ModelParameters.txt',[T,U_e,l1,l2,l3,Jp,Jt,Je,m1,m2,g,...
    k_amp, amp_sat_u, amp_sat_l, DAC_lim_u, DAC_lim_l,...
    e_lim_l, e_lim_u, p_lim_l, p_lim_u])
csvwrite('KF_Initial.txt',KF_Initial)
csvwrite('Kdi_Initial.txt',Kdi_Initial)