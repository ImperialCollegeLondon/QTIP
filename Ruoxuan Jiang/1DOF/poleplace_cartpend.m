clear all, close all, clc

m_p = 0.853;
l = 0.45;
L = 0.52;
g = -9.81;
% D = 0;
J = 1;
f = 1;

s = 1; % pendulum up (s=1)
A = [0,1;
    (-m_p*g*l)/J,-f*l];
B = [0; (L/J)];
C = [1 0];
D = [0];

eig(A);
rank(ctrb(A,B));  % is it controllable

% A = [0 1 0 0;
%     0 -d/M -m*g/M 0;
%     0 0 0 1;
%     0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
% 
% B = [0; 1/M; 0; s*1/(M*L)];
% eig(A)
% 
% rank(ctrb(A,B))  % is it controllable

%%  Pole placement
% p is a vector of desired eigenvalues
p = [-2.5; -0.7]; % good
K = place(A,B,p);
GoalPos = [pi;0];
% K = lqr(A,B,Q,R);


y0 = [pi+.1; 0];
tspan = 0:.01:10;
[t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,K,GoalPos),tspan,y0);

% tspan = 0:.01:20;
% % if(s==-1)
% %     y0 = [0; 0];
% %     [t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,K,[0; 0],0),tspan,y0);
% % %     [t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,K,[0; 0],0),tspan,y0);
% % %     [t,y] = ode45(@(t,y)cartpend(y,m_p,l,g,J,d1,d2,D,0,0),tspan,y0)
% % elseif(s==1)
% y0 = [pi+.1; 0];
% %     [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
% [t,y] = ode45(@(t,y)cartpend2(y,m_p,g,L,l,J,f,K,[pi;0],tspan,y0));
% % else 
% % end

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m_p,L);
end
