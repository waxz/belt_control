function plotResults(logger)

figure('Position',[100 50 1400 1000])

subplot(5,1,1)

plot(logger.time,logger.xTarget,'r--','LineWidth',2)
hold on
plot(logger.time,logger.xParcel,'b','LineWidth',2)
plot(logger.time,logger.xRef,'g:','LineWidth',1.5)

grid on

title('Position Tracking')
ylabel('Position (m)')

legend({'Target','Parcel','Reference'})

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(5,1,2)

plot(logger.time,logger.vTarget,'r--','LineWidth',2)
hold on
plot(logger.time,logger.vParcel,'b','LineWidth',2)
plot(logger.time,logger.vRef,'g:','LineWidth',1.5)

grid on

title('Velocity Tracking')
ylabel('Velocity (m/s)')

legend({'Target','Parcel','Reference'})

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(5,1,3)

plot(logger.time,logger.aRef,'LineWidth',2)

grid on

title('Acceleration')
ylabel('m/s^2')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(5,1,4)

h1 = plot(logger.time,logger.posError,'LineWidth',2);

hold on

h2 = plot(logger.time,logger.velError,'LineWidth',2);

grid on

title('Tracking Errors')
ylabel('Error')

legend([h1 h2], ...
    {'Position Error','Velocity Error'}, ...
    'Location','best')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(5,1,5)

plot(logger.time,1000*logger.dt,'LineWidth',1)

grid on

title('Measured Loop Time')
ylabel('dt (ms)')
xlabel('Time (s)')

end