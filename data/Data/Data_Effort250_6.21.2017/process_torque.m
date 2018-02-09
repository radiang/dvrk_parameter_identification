clear all
close all
pos=csvread('data_position.csv');
vel=csvread('data_velocity.csv');
tor=csvread('data_torque.csv');

hz=250;
Time=linspace(0,length(tor)-1,length(tor))*1/hz;

figure()
plot(Time,[pos(:,1) pos(:,2) pos(:,3)]);
title('Position')
legend('Joint 1[deg]', 'Joint 2[deg]', 'Joint 3[m]')
xlabel('Time (s)')
grid on

figure()
plot(Time,[vel(:,1) vel(:,2) vel(:,3)]);
title('Velocity')
legend('Joint 1[Rad/s]', 'Joint 2[Rad/s]', 'Joint 3[m/s]')
xlabel('Time (s)')

grid on

figure()
plot(Time,[tor(:,1) tor(:,2) tor(:,3)]);
title('Torque')
legend('Joint 1[N/m]', 'Joint 2[N/m]', 'Joint 3[N]')
xlabel('Time (s)')
grid on
axis tight