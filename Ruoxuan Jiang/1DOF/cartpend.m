function dy = cartpend(y,m_p,l,g,J,d1,d2,D,T1,T2)

Sy = sin(y(1));
Cy = cos(y(1));

% dy(1,1) = y(2);
% dy(2,1) = (D*Cy-T1+T2+d1-d2+m_p*g*Sy)/(J+m_p*l^2);

dy(1) = y(2);
dy(2) = (D*Cy-T1+T2+d1-d2+m_p*g*Sy)/(J+m_p*l^2);

% function dy = cartpend(y,m,M,L,g,d,u)
% Sy = sin(y(3));
% Cy = cos(y(3));
% D = m*L*L*(M+m*(1-Cy^2));
% dy(1,1) = y(2);
% dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
% dy(3,1) = y(4);
% dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;