clear;
clc;

x1=-2:0.1:2; 
y1=2+0.*x1; %% 产生一个和x1长度相同的y1数组 
plot(x1,y1)
hold on
y2=-5:1:5; 
x2=5+0.*y2; %% 产生一个和y2长度相同的x2数组 
plot(x2,y2)
hold on
%% 方法二：
% line([1,1],[min(y(:)) max(y(:))]);
line([1,1],[0,3])
hold on
%% 方法三：
plot([1.5 1.5],[1,2]);
hold on