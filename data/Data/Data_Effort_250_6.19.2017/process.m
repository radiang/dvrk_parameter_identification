clear all

vel=csvread('data_velocity.csv');
hz=250;
time=linspace(0,length(vel),length(vel)+1)*1/hz;

for i=1:length(vel)-1
    acc(i,1)=(vel(i+1,1)-vel(i,1))*hz;
    acc(i,2)=(vel(i+1,2)-vel(i,2))*hz;
    acc(i,3)=(vel(i+1,3)-vel(i,3))*hz;
end

figure(1)
plot(time(1:end-1),[vel(:,1) vel(:,2) vel(:,3)]);
title('velocity')

figure(2)
plot(time(1:end-2),[acc(:,1) acc(:,2) acc(:,3)]);
title('acceleration')

