%http://www.micromouseonline.com/2010/07/07/calibrating-reflective-sensors/
clear all

load('IR055.mat')
meanV(1) = mean(voltage)

load('IR073.mat')
meanV(2) = mean(voltage)

load('IR105.mat')
meanV(3) = mean(voltage)

load('IR145.mat')
meanV(4) = mean(voltage)

load('IR176.mat')
meanV(5) = mean(voltage)

load('IR207.mat')
meanV(6) = mean(voltage)

load('IR236.mat')
meanV(7) = mean(voltage)

load('IR273.mat')
meanV(8) = mean(voltage)

load('IR301.mat')
meanV(9) = mean(voltage)

load('IR341.mat')
meanV(10) = mean(voltage)

load('IR361.mat')
meanV(11) = mean(voltage)

load('IR395.mat')
meanV(12) = mean(voltage)


distance = [55; 73; 105; 145; 176; 207; 236; 273; 301; 341; 361; 395];

plot(distance,meanV)