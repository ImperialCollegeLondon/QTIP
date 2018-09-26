%http://www.micromouseonline.com/2010/07/07/calibrating-reflective-sensors/
clear all

load('IRDist055mm.mat')
meanV(1) = mean(voltage)

load('IRDist080mm.mat')
meanV(2) = mean(voltage)

load('IRDist110mm.mat')
meanV(3) = mean(voltage)

load('IRDist243mm.mat')
meanV(4) = mean(voltage)

load('IRDist322mm.mat')
meanV(5) = mean(voltage)

load('IRDist361mm.mat')
meanV(6) = mean(voltage)

distance = [55; 80; 110; 243; 322; 361];

plot(distance,meanV)