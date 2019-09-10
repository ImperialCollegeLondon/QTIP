clear all, close all, clc
m_p = 0.853;
l = 0.35;
L = 0.52;
g = -9.81;
% D = 0;
J = 0.0215;
f = 0.0005;

s = 1; % pendulum up (s=1)
A = [0,1;
    (-m_p*g*l)/J,0];
B = [0; (L/J)];
C = [1 0];
D = [0];
eig(A);
%% system
%sys = ss(A,B,C,D);
Q = [1 0;0 10];
R = 0.0001; %increasing R making the cost more expensive and less aggressive
%%
det(ctrb(sys));
det(obsv(sys));
% rlocus(sys);
%%
%K = lqr(A,B,Q,R);
GoalPos = [pi;0];
x0 = [pi+.1; 0];
y0 = [pi+.1; 0];
tspan = 0:.01:10;
[t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,K,GoalPos),tspan,y0);

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m_p,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)