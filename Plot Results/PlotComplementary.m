
box on
hold on
p1 = plot(data(:,1)/1000,data(:,2),data(:,1)/1000,data(:,3), data(:,1)/1000,data(:,4),'LineWidth',1.5)
xlim([0 10])
xl = xlabel('Time (s)')
yl = ylabel('Roll (degrees)')
hl = legend(p1, 'Accelerometer', 'Gyro', 'Roll');
set(hl, 'Interpreter', 'latex')
set(xl, 'Interpreter', 'latex')
set(yl, 'Interpreter', 'latex')
