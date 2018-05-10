function [fs,gen]=check_fourier(gen,ident,traj,fs);
close all
test.v= reshape(fs.vars,[],gen.dof).';
test.w = fs.w;


test.dof = gen.dof;
test.time = fs.time;
test.N=fs.Nl;



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
[fs.qi, fs.qdi,fs.qddi]= fourier_f(test);


%% Condition number of regressor
for i=1:length(fs.qi(1,:)) 
    W(1+(i-1)*gen.dof:gen.dof+(i-1)*gen.dof,:)=gen.condfun(fs.qi(1,i),fs.qi(2,i),fs.qi(3,i),fs.qdi(1,i),fs.qdi(2,i),fs.qdi(3,i),fs.qddi(1,i),fs.qddi(2,i),fs.qddi(3,i));
 %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end

fs.cond=cond(W);
%% plot

figure()
plot(test.time,fs.qi(1,:),test.time,fs.qi(2,:),test.time,fs.qi(3,:));
legend('1','2','3')
title('Joint Positions')
ylabel('rad')
xlabel('time')

figure()
plot(test.time,fs.qdi(1,:),test.time,fs.qdi(2,:),test.time,fs.qdi(3,:));
legend('1','2','3')
title('Joint Velocities')
ylabel('rad/s')
xlabel('time')

ass = [fs.qi;fs.qdi;fs.qddi];

csvname=strcat('data/',gen.filename,'/',gen.fourfilename,'.csv');
csvwrite(csvname,ass);

end