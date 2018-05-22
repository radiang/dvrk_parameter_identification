function [gen,traj,ident]=new_par_ident(gen,traj,ident,fs,plot_on)
% filename = 'new_3dof_inaxis_svd';
% loadname = strcat('data/',filename,'_optimized.mat');
% load(loadname);
close all

foldername=strcat('data/',gen.filename,'/');
%testname = 'PID_data2';
csvname = strcat(foldername,gen.csvfilename,'_results.csv');
q=csvread(csvname);

t = linspace(1,length(q(:,1)'),length(q(:,1)')); 
dof_num = gen.dof;


%% Delete close to zero velocity
% tol = 0.08; 
% %Deleting data near zero friction better results for joint 1 and 2, 
% %but makes joint 3 data worse.
% 
% for i = length(q(:,1)):-1:1
%     if abs(q(i,4))<tol||abs(q(i,5))<tol||abs(q(i,6))<tol
%         q(i,:) = [];
%         t(i) = [];
%       
%     end
% end

%% Filter Velocity

% windowSize = ident.window; 
% b = (1/windowSize)*ones(1,windowSize);
% a = ident.a;

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

for i=1:length(qdf(:,1)')-1
   acc(i,1:3) = (qdf(i+1,1:3)-qdf(i,1:3))/traj.ts; 
end
%% Process Torque Data

ident.tau=q(:,7:9);

ident.scale = ones(1,gen.dof);
ident.scale= 1./max(abs(ident.tau));

%ident.tau = (diag(ident.scale)*ident.tau.').';

%windowSize = ident.window; 
%b = (1/windowSize)*ones(1,windowSize);
%a = ident.a;


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

%% Delete close to zero velocity
tol = 0.005; 
% Deleting data near zero friction better results for joint 1 and 2, 
% but makes joint 3 data worse.
% 
% for i = length(q(:,1)):-1:1
%     if abs(q(i,4))<tol||abs(q(i,5))<tol||abs(q(i,6))<tol
%         q(i,:) = [];
%         t(i) = [];
%         qdf(i,:) = [];
%         acc(i,:) = [];
%         tauf(i,:) = [];
%     end
% end

%% Plot looksee
if plot_on ==1
figure()
subplot(2,2,1)
plot(t,q(:,1)')
hold on
plot(t,q(:,2)')
hold on 
plot(t,q(:,3)')
title('Positions')
legend('joint 1','joint 2','joint 3')

subplot(2,2,2)
plot(t,qdf(:,1)')
hold on
plot(t,qdf(:,2)')
hold on 
plot(t,qdf(:,3)')
title('Velocities')
legend('joint 1','joint 2','joint 3')

subplot(2,2,3)
plot(t(1:end-1),acc(:,1)')
hold on
plot(t(1:end-1),acc(:,2)')
hold on 
plot(t(1:end-1),acc(:,3)')
title('Accelerations')
legend('joint 1','joint 2','joint 3')

subplot(2,2,4)
plot(t,tauf(:,1)')
hold on
plot(t,tauf(:,2)')
hold on 
plot(t,tauf(:,3)')
title('Torque')
legend('joint 1','joint 2','joint 3')
end


%% 
N = length(q(:,1))-10;
for i=1:N 
    W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=gen.condfun(q(i,1),q(i,2),q(i,3),qdf(i,1),qdf(i,2),qdf(i,3),acc(i,1),acc(i,2),acc(i,3));
end

%X = ones(1,length(gen.Par2));
X = 1./vecnorm(W);
P = diag(X);

ident.W= W*P;

gen.LS_cond= cond(ident.W);
ident.wtau = reshape(ident.tau.',1,[]).';

gen.ls_par2 = pinv(ident.W)*(ident.wtau(1:length(ident.W)));
gen.ls_par2 = P*gen.ls_par2;
%gen.ls_par2 = gen.ls_par2;

% Save
ident.P = P;
ident.qddi = acc.';
ident.qdi = qdf.';
ident.qi = q(:,1:3).';
ident.q=q;

ident.t = t;


%% Weighted Least Squares

% Coefficients

Wl = zeros(round(N/3)+10,length(gen.Par2),gen.dof);
for i=1:N 
    Wl(i,:,1) = W(1+(i-1)*dof_num,:);
    Wl(i,:,2) = W(2+(i-1)*dof_num,:);
    Wl(i,:,3) = W(3+(i-1)*dof_num,:);
end

for j = 1:gen.dof
    var2(j) = norm(ident.tau(1:N,j)-Wl(:,:,j)*gen.ls_par2)^2/(N-length(gen.Par2)) ;
end

r_var2 = 1./var2;
%r_var2 = ident.scale;
huge = [];

for i = 1:N
   huge = [huge,r_var2];
end

G = diag(huge);

ident.r_var2 = r_var2;
% Least Squares Solution
gen.wls_cond= cond(G*ident.W);
Wwtau = G*ident.wtau(1:length(ident.W)); 

gen.wls_par2 = pinv(G*ident.W)*(Wwtau);
gen.wls_par2 = P*gen.wls_par2;


%ident.wls_G = G;
end






