clear all, close all, clc
m_p = 0.853;
l = 0.35;
L = 0.52;
g = -9.81;
    % D = 0;
J = 0.077;
GoalPos = [0;0];

f = 0.0005;
k_p = 0.0074;

s = 1; % pendulum up (s=1)
A = [0,1;
    (m_p*g*l)/J,0];
B = [0; L/J];
C = [1 0];
D = [0];
eig(A);
rank(ctrb(A,B));
%% system
sys = ss(A,B,C,D);
% nonlinear
Q = [100 0;0 10];
R = 1000; %increasi0ng R making the cost more expensive and less aggressive
% linear
% Q = [100 0;0 10];
% R = 1000;
%%
det(ctrb(sys));
det(obsv(sys));
% rlocus(sys);
%%
K = lqr(A,B,Q,R);
GoalPos = [0;0];
x0 = [0.1; 0];
y0 = [0.1; 0];