clear all

sim('QTIP1DOFCartAnd1DOFPendulumPID')

p1 = plot(SimulinkResponse,'LineWidth',1.5)
xlim([0 10])
ylim([-3 3])
xl = xlabel('Time (s)')
yl = ylabel('Position (m) / Angle (degrees)')
title('')
hl = legend(p1, 'Position $x''$', 'Pitch $\theta$');
set(hl, 'Interpreter', 'latex')
set(xl, 'Interpreter', 'latex')
set(yl, 'Interpreter', 'latex')

figure; p2 = plot(SimscapeResponse,'LineWidth',1.5)
xlim([0 10])
ylim([-3 3])
xl = xlabel('Time (s)')
yl = ylabel('Position (m) / Angle (degrees)')
title('')
h2 = legend(p2, 'Position $x''$', 'Pitch $\theta$');
set(h2, 'Interpreter', 'latex')
set(xl, 'Interpreter', 'latex')
set(yl, 'Interpreter', 'latex')