clear all 
close all

filename = '3dof_inplanepitch_svd';
loadname = strcat('data/',filename,'_optimized.mat');
load(loadname);

q=csvread('data/PID_data_.75speed_try2.csv');
t = linspace(1,length(q(:,1)'),length(q(:,1)')); 

%% Plot looksee
figure(1)
plot(t,q(:,4)')
hold on
plot(t,q(:,5)')
hold on 
plot(t,q(:,6)')
title('Velocities')
legend('joint 1','joint 2','joint 3')

%% Filter 

windowSize = 12; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
x1= q(:,4).';
x2= q(:,5).';
x3= q(:,6).';

qdf(:,1) = filter(b,a,x1)';
qdf(:,2) = filter(b,a,x2)';
qdf(:,3) = filter(b,a,x3)';

% plot(t,x)
% hold on
% plot(t,y)
% legend('Input Data','Filtered Data')
% hold off



%% Get Derivative

for i=1:length(qdf(:,1)')-1
   acc(i,1:3) = qdf(i+1,1:3)-qdf(i,1:3); 

end
%% Plot Acceleration  

figure(2)
plot(t(1:end-1),acc(:,1)')
hold on
plot(t(1:end-1),acc(:,2)')
hold on 
plot(t(1:end-1),acc(:,3)')
title('Accelerations')
legend('joint 1','joint 2','joint 3')
%% 
for i=1:length(q(:,1))-10 
 W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end

pinv(W);

tau = reshape(q(:,7:9).',1,[]);
Par_num = pinv(W)*tau(1:length(W)).'

%% Save

savename=strcat('data/',filename,'_results.mat');
save(savename,'W','Q','Qd','Qdd','Ys2','q','acc','tau');



