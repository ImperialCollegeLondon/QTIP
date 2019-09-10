clear all, close all, clc
m_p = 0.853;
l = 0.35;
L = 0.52;
g = -9.81;
% D = 0;
J = 0.0077;
f = 0.0005;
k_p = 0.0074;
% decoupled
A = [0,1;
    -g/l,0];
B = [0; 1/(m_p*l^2)];
C = [1 0];
D = [0];

%% system
sys = ss(A,B,C,D);
Q = [100 0;0 100];
R = 10; %increasing R making the cost more expensive and less aggressive
Q2 = [100 0;0 100];
R2 = 10;
%%
det(ctrb(sys));
det(obsv(sys));
% rlocus(sys);
%%
K = lqr(A,B,Q,R);
K2 = lqr(A,B,Q2,R2);
GoalPos = [0;0];
x0 = [0.1; 0];
y0 = [0.1; 0];