function drawcartpend_bw(y,m_p,L)
x = 0;
th = y(1);

% kinematics
% x = 3;        % cart position
% th = 3*pi/2;   % pendulum angle

% dimensions
% *0.265
W = 0.212;  % cart width
H = 0.1325; % cart height
wr = 0.053; % wheel radius
mr = 0.0795*sqrt(m_p); % mass radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px = x + L*sin(th);
py = y - L*cos(th);

plot([-10 10],[0 0],'w','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
rectangle('Position',[x-wr/2,y+H/2-wr/2,wr,wr],'Curvature',1,'FaceColor',[1 1 0.1],'EdgeColor',[1 1 1])

plot([x px],[y+H/2 py],'w','LineWidth',5)
al = atan(0.19875/L);
l1 = L/cos(al);
plot([x+l1*sin(th-al) x+l1*sin(th+al)],[y-l1*cos(th-al) y-l1*cos(th+al)],'g','LineWidth',5)
be = atan(0.19875/0.6625);
l2 = 0.6625/cos(be);
l3 = 0.19875/cos(be);
% be = atan(0.75/2.5);
% l2 = 2.5/cos(be);
% l3 = 0.75/cos(be);
% plot([x+l3*sin(th+2*al-be+0.7) x+l2*sin(th+be)],[y-l3*cos(th+2*al-be) y-l2*cos(th+be)],'g','LineWidth',5)
% plot([x+l3*sin(th-2*al+be-0.7) x+l2*sin(th-be)],[y-l3*cos(th-2*al+be) y-l2*cos(th-be)],'g','LineWidth',5)

% rectangle('Position',[px-0.75,py-0.025,1.5,0.05],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
% rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
% xlim([-4 4]);
% ylim([-4 4]);
set(gca,'Color','k','XColor','w','YColor','w')
% set(gca,'xlim',[-4,4])
% set(gca,'ylim',[-4,4])
axis([-1 1 -1 1])
% set(gcf,'Position',[10 900 800 400])
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   
% box off
drawnow
hold off