function [fs,gen,test]=check_fourier(gen,traj,fs)
close all
test.v= reshape(fs.vars,[],gen.dof).';
test.w = fs.w;

%%%%%%%%%%%%%%%%%%%%%%%
test.ts = 0.005;
%%% %%%%%%%%%%%%%%%%%%%

test.dof = gen.dof;
test.N=fs.Nl;

test.period=fs.period;
test.disc_num = test.period./test.ts;
test.time = linspace(0,test.period-test.ts,test.disc_num);

%Cheating
% test.v(:,:) = [0.05 -0.29 0.48 0.55 0.65 0.19 -0.4 -0.18 0.63 -0.46 -0.29;
%                 0.03 0.29 -0.23 0.32 0.82 0.09 -0.08 0.05 -0.02 0.65 0.11;
%                 -0.07 0.4 0.45 0.40 -0.03 -0.49 0.32 -0.26 -0.63 0.06 0.04];
%             
% scale = [0.98, 1, 0.1].';
% test.v(:,:) = test.v(:,:).*scale;
%                    
% test.w = 0.1*pi();
% fs.period = 30;
% fs.ts = 0.02;
% 
% fs.disc_num = fs.period/fs.ts;
% fs.time = linspace(0,fs.period-fs.ts,fs.disc_num);

%% Input
[test.qi, test.qdi,test.qddi]= fourier_f(test);


%% Condition number of regressor
for i=1:length(test.qi(1,:)) 
    X = gen.condfun(test.qi(1,i),test.qi(2,i),test.qi(3,i),test.qdi(1,i),test.qdi(2,i),test.qdi(3,i),test.qddi(1,i),test.qddi(2,i),test.qddi(3,i));
    W(1+(i-1)*gen.dof:gen.dof+(i-1)*gen.dof,:)= X.*[0.5200,    0.5034,    0.2574].';
    
    %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end
P = diag(1./vecnorm(W));

test.test_cond_weighted=cond(W*P);
%% plot

figure()
plot(test.time,test.qi(1,:),test.time,test.qi(2,:),test.time,test.qi(3,:));
legend('1','2','3')
title('Joint Positions')
ylabel('rad')
xlabel('time')

figure()
plot(test.time,test.qdi(1,:),test.time,test.qdi(2,:),test.time,test.qdi(3,:));
legend('1','2','3')
title('Joint Velocities')
ylabel('rad/s')
xlabel('time')

ass = [test.qi;test.qdi;test.qddi];

csvname=strcat('data/',gen.filename,'/',gen.fourfilename,'.csv');
csvwrite(csvname,ass);

end