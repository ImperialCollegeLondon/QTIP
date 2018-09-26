
box on
hold on
p1 = plot(data(:,1)/1000,data(:,4),data(:,1)/1000,data(:,6),'LineWidth',1.5)
xlim([0 15])
xl = xlabel('Time (s)')
yl = ylabel('Roll (degrees)')
hl = legend(p1, 'DMP', 'Complementary');
set(hl, 'Interpreter', 'latex')
set(xl, 'Interpreter', 'latex')
set(yl, 'Interpreter', 'latex')
