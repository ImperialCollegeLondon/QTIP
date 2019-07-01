% this script is written to work with MATLAB R2018b - JYP
clear variables
close all

%% perhaps these can all be loaded from a mat file
% define parameters and constants
g = 9.81;   % gravitational constant
r = 0.35;   % distance from pivot point to pendulum's centre of gravity
m = 1.066;  % mass of pendulum
K = 0.0074; % assumed that thrust for each rotors at the centre of gravity f_i = K*u_i for PWM duty cycle u_i

% define linearised state space matrices - see calculations from livescript
A = zeros(4,4);
A(1,2) = 1;
A(2,1) = g/r;
A(3,4) = 1;
A(4,3) = g/r;

B = zeros(4,2);
B(2,1) = K/(m*r^2);
B(4,2) = K/(m*r^2);

% C = zeros(2,4);
% C(1,1) = 1;
% C(2,3) = 1;
C = eye(4);

D = zeros(4,2);

% define sampling period
Ts = 0.015;

Q = diag([100 20 100 20]);
R = 0.1*eye(2);
N = zeros(4,2);

[Kd,Sd,ed] = lqrd(A,B,Q,R,Ts);
[Ks,Ss,es] = lqr(A,B,Q,R);

% clsys = feedback(sys,K);
% clsysd = feedback(sysd, K);
% Tstop = 10;
% x0 = [0.2; 0; 0.2; 0];
% initial(clsysd,x0,Tstop)


%% save controllers to mat file
save('linearSSModel.mat','g','r','m','Kd','A','B','C','D');
