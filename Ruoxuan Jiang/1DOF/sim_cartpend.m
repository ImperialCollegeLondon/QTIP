clear all, close all, clc

m_p = 0.853;
l = 0.45;
L = 0.52;
g = -9.81;
% D = 0;
J = 1;
f = 1;

% GoalPos = [pi; 0]

tspan = 0:.1:20;
y0 = [pi; .5]; %constraints
[t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,0),tspan,y0);

for k=1:length(t)
    drawcartpend_bw(y(k,:),m_p,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)