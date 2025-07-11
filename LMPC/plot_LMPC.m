% Plot the solution
X = reshape(out.x,[size(out.x,3),1]);
Y = reshape(out.y,[size(out.y,3),1]);
refe_x = reshape(out.reference_x,[size(out.reference_x,3),1])
refe_y = reshape(out.reference_y,[size(out.reference_y,3),1])



figure
subplot(3,1,1),plot(out.u1,'LineWidth',1.5)
xlim([0 257])
grid on
xlabel('Time [s]')
ylabel('Thrust [N]')
legend('$u_1$ ','interpreter','latex')
title('Thrust 1')
subplot(3,1,2),plot(out.u2,'LineWidth',1.5)
xlim([0 257])
grid on
xlabel('Time [s]')
ylabel('Thrust [N]')
legend('$u_1$ ','interpreter','latex')
title('Thrust 2')
subplot(3,1,3),plot(out.u3,'LineWidth',1.5)
xlim([0 257])
grid on
xlabel('Time [s]')
ylabel('Torque [N/m]')
legend('$u_1$ ','interpreter','latex')
title('Thrust 3')
meancomp=mean(out.t_comp(2:end))
figure
plot(out.t_comp(2:end),'LineWidth',1.5),hold on,
yline(meancomp,'--k','LineWidth',2);
grid on
xlabel('Time [s]')
ylabel('Solving time [s]')
legend('Solving time','Mean','interpreter','latex')
title('Solving time')

figure
plot(X,Y,'LineWidth',1.5)
hold on
plot(refe_x,refe_y,'LineWidth',1.5)
xlim([0 out.tout(end)])
grid on
xlabel('x [m]')
ylabel('y [m]')
legend('$Trajectory$ ','Reference','interpreter','latex')
title('Trajectory')

error = sqrt((refe_x-X).^2+(refe_y-Y).^2)

figure
plot(error,'LineWidth',1.5)
xlim([0 out.tout(end)])
grid on
xlabel('Distance error [m]')
ylabel('Time [s]')
legend('$Error$ ','interpreter','latex')
title('Trajectory error')

MSE_x = immse(X,refe_x)
MSE_y = immse(Y,refe_y)
