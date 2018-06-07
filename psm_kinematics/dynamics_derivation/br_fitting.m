% BCD parameter estimation f
function [gen,ctrl] = br_fitting(gen,ctrl)

close all 

foldername=strcat('data/',gen.filename,'/');
%testname = 'PID_data2';
csvname = strcat(foldername,'brtest_results.csv');
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


%% Filter Velocity

 fc = 12;
 fss = 200;

%fc = 3;
%fss = 400;

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
legend('before','after');
savefig = strcat('pictures/br_test_',gen.fourfilename,'b_a.png');
saveas(gcf,savefig)

figure()
scatter(qdf(1:N,3),distortion(3,:),10);
title('Friction torques')
xlabel('Velocity [rad/s]')
ylabel('Torques [N m]')

savefig = strcat('pictures/br_test_',gen.fourfilename,'Fr_vel.png');
saveas(gcf,savefig)
% %%
% xdata = qdf(1:N,3).';
% ydata = distortion(3,:);
% 
% Fc_init      = 0.1;
% Fs_init      = 0.5;
% vs_init      = 0.007;
% delta_init   = 0.5;
% sig_init = 0.45;
% 
%     f = fittype('A+(V-A)*exp(-(x/C)^D)+E*x'); 
%     [fit1,gof,fitinfo] = fit(xdata.',ydata.',f,'StartPoint',[Fc_init Fs_init vs_init delta_init sig_init]);
%    
%     
%        
% Bf = fit1.b
% Cf = fit1.c
% Df = fit1.d;
% 
% Br = fit1.b
% Cr = fit1.c
% Dr = fit1.d
% 
% x = (S(37).Slip_Favg(3:end))';
% y = Df*sin(Cf*atan(Bf*x));
% 
% y1 = (S(37).F_Favg(2,3:end))';
% 
% xr = (S(37).Slip_Ravg(3:end))';
% yr = Dr*sin(Cr*atan(Br*xr));
% 
% y1r = (S(37).F_Ravg(2,3:end))';
% 
% figure;
% scatter(x,y);
% hold on
% scatter(x,y1);
% legend('Estimated Ffy ', 'Measured Ffy');
% xlabel('Front tire slip angle');
% ylabel('Lateral Force on Front Tire');
% 
% figure;
% scatter(xr,yr);
% hold on
% scatter(xr,y1r);
% legend('Estimated Fry ', 'Measured Ffy');
% xlabel('Rear tire slip angle');
% ylabel('Lateral Force on Rear Tire');
% 

end


% fit1 = 
% 
%      General model:
%      fit1(x) = d*sin(c*atan(b*x))
%      Coefficients (with 95% confidence bounds):
%        b =       10.43  (10.28, 10.58)
%        c =       1.337  (1.329, 1.346)
%        d =        1376  (1374, 1379)