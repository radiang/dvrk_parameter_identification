% BCD parameter estimation f
function [gen,ctrl] = br_fitting(gen,ctrl)

close all 

foldername=strcat('data/',gen.filename,'/');
%testname = 'PID_data2';
csvname = strcat(foldername,'brtest2_results_psm2.csv');
q1=csvread(csvname);


  
q = [q1];

 %% Delete nonlinear data

%  iteration = 241;
% nonlinear = 41;
% A = iteration-nonlinear;
% 
% for i = 1:30
% q((i-1)*A+1:(i-1)*A+nonlinear,:) = [];
% %t((i-1)*A+1:(i-1)*A+nonlinear) = [];
% end
% 
% 
% t = linspace(1,length(q(:,1)'),length(q(:,1)'));
% 


for i = length(q):-1:1
   if (q(i,1)==0)
      q(i,:) = []; 
   end
end


t = linspace(1,length(q(:,1)'),length(q(:,1)'));
t = t*.1/200;

%% Filter Velocity

 fc = 12;
 fss = 200;


[b,a] = butter(10,fc/(fss/2));

x1= q(:,4).';
x2= q(:,5).';
x3= q(:,6).';

qdf(:,1) = filtfilt(b,a,x1)';
qdf(:,2) = filtfilt(b,a,x2)';
qdf(:,3) = filtfilt(b,a,x3)';

figure()
subplot(3,1,1)
plot(t,x1,t,qdf(:,1)');
title('Velocity Filter')
subplot(3,1,2)
plot(t,x2,t,qdf(:,2)');
subplot(3,1,3)
plot(t,x3,t,qdf(:,3)');
ylabel('Velocity [m/s]')
xlabel('time [s]')

savefig = strcat('pictures/br_test_',gen.fourfilename,'Velocity_filter.png');
saveas(gcf,savefig)

%% Get Derivative
sample_time = 1/100;
for i=1:length(qdf(:,1)')-1
   acc(i,1:3) = (qdf(i+1,1:3)-qdf(i,1:3))./sample_time; 
end
%% Process Torque Data

ident.tau=q(:,7:9);

ident.scale = ones(1,gen.dof);
ident.scale= 1./max(abs(ident.tau));

%ident.tau = (diag(ident.scale)*ident.tau.').';


x1= ident.tau(:,1).';
x2= ident.tau(:,2).';
x3= ident.tau(:,3).';

 fc = 6;
 fs = 200;

%fc = 2;
%fss = 400;

[b,a] = butter(8,fc/(fss/2));

tauf(:,1) = filtfilt(b,a,x1)';
tauf(:,2) = filtfilt(b,a,x2)';

% fc = 2;
% fss = 400;

[b,a] = butter(8,fc/(fss/2));
tauf(:,3) = filtfilt(b,a,x3)';

ident.tauf = tauf;
%%%%%%%%%%
%ident.tau = tauf;
%%%%%%%%%%%
figure()
subplot(3,1,1)
plot(t,x1,t,tauf(:,1)');
title('Torque Filter')
subplot(3,1,2)
plot(t,x2,t,tauf(:,2)');
subplot(3,1,3)
plot(t,x3,t,tauf(:,3)');
ylabel('Force [N]')
xlabel('time [s]')

savefig = strcat('pictures/br_test_',gen.fourfilename,'torque_filter.png');
saveas(gcf,savefig)


%% Delete Non friction Torques
minus = matlabFunction(ctrl.num_Y);
gravity = matlabFunction(ctrl.G);

N = length(q(:,1))-10;
for k = 1:N
    A = minus(q(k,1),q(k,2),q(k,3),qdf(k,1),qdf(k,2),qdf(k,3),tauf(k,1),tauf(k,2),tauf(k,3));
    B = gravity(q(k,1),q(k,2),q(k,3));
    distortion(1:3,k) = tauf(k,:).'-(A-B);
end

%dirty = minus(q
figure()
plot(t(1:N),tauf(1:N,3)',t(1:N),distortion(3,1:N));
xlabel('Time [s]')
ylabel('Force [N]')
title('Force Ramp Up Test: Delete Torque')

legend('before','after');
savefig = strcat('pictures/br_test_',gen.fourfilename,'b_a.png');
saveas(gcf,savefig)

figure()
scatter(qdf(1:N,3),distortion(3,:),10);
title('Force Ramp Up Test')
xlabel('Velocity [m/s]')
ylabel('Force [N]')

savefig = strcat('pictures/br_test_',gen.fourfilename,'Fr_vel.png');
saveas(gcf,savefig)

end

