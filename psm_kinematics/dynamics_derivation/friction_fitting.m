% BCD parameter estimation f
function [gen,ctrl] = friction_fitting(gen,ctrl)


foldername=strcat('data/',gen.filename,'/');
%testname = 'PID_data2';
csvname = strcat(foldername,gen.csvfilename,'_results2.csv');
q=csvread(csvname);

t = linspace(1,length(q(:,1)'),length(q(:,1)')); 

%% Filter Velocity

 fc = 3.5;
 fss = 200;

%fc = 3;
%fss = 400;

[b,a] = butter(8,fc/(fss/2));

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

 fc = 3.5;
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

%% Delete Non friction Torques
minus = matlabFunction(ctrl.num_Y);
N = length(q(:,1))-10;
for k = 1:N
    distortion(1:3,k) = tauf(k,:).'- minus(q(k,1),q(k,2),q(k,3),qdf(k,1),qdf(k,2),qdf(k,3),tauf(k,1),tauf(k,2),tauf(k,3));
end
%dirty = minus(q
figure()
subplot(3,1,1)
plot(t(1:N),tauf(1:N,3)',t(1:N),distortion(3,1:N));
legend('before','after');
%%
b_init = 13;
c_init = 1.5;
d_init = 1000;

b_init1 = 13;
c_init1 = 1.5;
d_init1 = 1000;

for i = 1:35
    %Paramter Estimation for Front Wheel
    %xdata =
    %ydata = 
    f = fittype('d*sin(c*atan(b*x))'); 
    [fit1,gof,fitinfo] = fit(xdata,ydata,f,'StartPoint',[b_init c_init d_init]);
    b_init = fit1.b;
    c_init = fit1.c;
    d_init = fit1.d;
    

    % Parameter Estimation for rear wheel 
    xdata1 = (S(i).Slip_Ravg(3:end))'; 
    ydata1 = (S(i).F_Ravg(2,3:end))';
    f2 = fittype('dr*sin(cr*atan(br*x))'); 
    [fit2,gof2,fitinfo2] = fit(xdata1,ydata1,f2,'StartPoint',[b_init1 c_init1 d_init1]);
    b_init1 = fit2.br;
    c_init1 = fit2.cr;
    d_init1 = fit2.dr;
    
       
end

Bf = fit1.b
Cf = fit1.c
Df = fit1.d;

Br = fit1.b
Cr = fit1.c
Dr = fit1.d

x = (S(37).Slip_Favg(3:end))';
y = Df*sin(Cf*atan(Bf*x));

y1 = (S(37).F_Favg(2,3:end))';

xr = (S(37).Slip_Ravg(3:end))';
yr = Dr*sin(Cr*atan(Br*xr));

y1r = (S(37).F_Ravg(2,3:end))';

figure;
scatter(x,y);
hold on
scatter(x,y1);
legend('Estimated Ffy ', 'Measured Ffy');
xlabel('Front tire slip angle');
ylabel('Lateral Force on Front Tire');

figure;
scatter(xr,yr);
hold on
scatter(xr,y1r);
legend('Estimated Fry ', 'Measured Ffy');
xlabel('Rear tire slip angle');
ylabel('Lateral Force on Rear Tire');


end


% fit1 = 
% 
%      General model:
%      fit1(x) = d*sin(c*atan(b*x))
%      Coefficients (with 95% confidence bounds):
%        b =       10.43  (10.28, 10.58)
%        c =       1.337  (1.329, 1.346)
%        d =        1376  (1374, 1379)