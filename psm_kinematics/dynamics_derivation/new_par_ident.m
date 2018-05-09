function [gen,traj,ident]=new_par_ident(gen,traj,ident,plot_on)
% filename = 'new_3dof_inaxis_svd';
% loadname = strcat('data/',filename,'_optimized.mat');
% load(loadname);

foldername=strcat('data/',gen.filename,'/');
%testname = 'PID_data2';
csvname = strcat(foldername,gen.csvfilename,'.csv');
q=csvread(csvname);

t = linspace(1,length(q(:,1)'),length(q(:,1)')); 
dof_num = gen.dof;
%% Plot looksee
if plot_on ==1
figure(1)
plot(t,q(:,4)')
hold on
plot(t,q(:,5)')
hold on 
plot(t,q(:,6)')
title('Velocities')
legend('joint 1','joint 2','joint 3')
end
%% Filter 

windowSize = ident.window; 
b = (1/windowSize)*ones(1,windowSize);
a = ident.a;
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
   acc(i,1:3) = (qdf(i+1,1:3)-qdf(i,1:3))/traj.ts; 
end
%% Plot Acceleration  
if plot_on==1
figure(2)
plot(t(1:end-1),acc(:,1)')
hold on
plot(t(1:end-1),acc(:,2)')
hold on 
plot(t(1:end-1),acc(:,3)')
title('Accelerations')
legend('joint 1','joint 2','joint 3')
end

%% 
for i=1:length(q(:,1))-10 
    W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=gen.condfun(q(i,1),q(i,2),q(i,3),qdf(i,1),qdf(i,2),qdf(i,3),acc(i,1),acc(i,2),acc(i,3));
    
  
 %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end

pinv(W);

ident.W= W;
ident.tau=q(:,7:9);

tau = reshape(q(:,7:9).',1,[]);
gen.par_cond=cond(W);
gen.par_num = pinv(W)*tau(1:length(W)).';

ident.acc = acc;
ident.qdf = qdf;
ident.q=q;


end






