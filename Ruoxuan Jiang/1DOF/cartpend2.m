function dydt = cartpend2(y,m_p,g,L,l,J,f,K,GoalPos)
% function dydt = cartpend2(y,m_p,g,L,l,J,f,u)
dydt = zeros(2,1);
% Sy = sin(y(1));
% Cy = cos(y(1));
u = -K*(y-GoalPos);

dydt(1) = y(2);
% dydt(2) = (D*Cy-T1+T2+d1-d2+m_p*g*Sy)/(J+m_p*l^2);
dydt(2) = (u*L-m_p*g*l*sin(y(1))-f*(y(2)^2)*l^3)/J;