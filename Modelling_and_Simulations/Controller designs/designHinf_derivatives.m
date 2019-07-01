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

C = eye(4);
D = zeros(4,2);

% define sampling period
Ts = 0.015;
% define control computation delay
Td = 0.015;

%% H-infinity optimal controller
% compute transfer matrix G
%[z,p] = ss2tf(A,B,C,D,1);

sss = ss(A,B,C,D,'StateName',{'roll','roll velocity','pitch','pitchvelocity'},'InputName',{'thrust x','thrust y'},'OutputName',{'roll','roll velocity','pitch','pitchvelocity'});
ssd = c2d(sss,Ts);
tfs = tf(sss);
tfd = tf(ssd);

% 2 input u
% 4 state x
% 4 output y
% 4 reference r
% 4 error z1 = e = r-y
% 2 input z2 = u
% 4 output z3 = y

% define cost weights
W1 = eye(4); % error cost
W2 = eye(2); % input energy cost
W3 = []; % output/state cost

% compute augmented plant transfer matrix
Ps = augw(sss,W1,W2,W3);
Pd = augw(ssd,W1,W2,W3);

% compute H-inf optimal controller
nmeas = 2;
ncont = 2;
[Khinfs,CL,gammac] = hinfsyn(Ps,nmeas,ncont);
[Khinfd,gammad] = sdhinfsyn(Ps,nmeas,ncont,'Ts',Ts,'Delay',1);

%% save controllers to mat file
%save('linearSSModel.mat','g','r','m','K','A','B','C','D');
%save('controllerMatrices.mat','Klqr','Khinf');
