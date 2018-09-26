clear all

sim('Pendulum3DOF')

p1 = plot(simout,'LineWidth',1.5)
xlim([0 10])
ylim([2.4 4])
xl = xlabel('Time (s)')
yl = ylabel('Angle (radians)')
title('')
hl = legend(p1, 'Roll $\phi$', 'Pitch $\theta$', 'Yaw $\psi$');
set(hl, 'Interpreter', 'latex')
set(xl, 'Interpreter', 'latex')
set(yl, 'Interpreter', 'latex')
