clear all
pos=csvread('data_position.csv');
tor=csvread('data_torque.csv');

hz=250;
Time=linspace(0,length(tor)-1,length(tor))*1/hz;

figure()
plot(Time,[pos(:,1) pos(:,2) pos(:,3)]);
title('data position')
legend('joint 1', 'joint2','joint3')
xlabel('Time (s)')
grid on

figure()
plot(Time,[tor(:,1) tor(:,2) tor(:,3)]);
title('data torque')
legend('joint 1', 'joint2','joint3')
xlabel('Time (s)')
grid on
axis tight