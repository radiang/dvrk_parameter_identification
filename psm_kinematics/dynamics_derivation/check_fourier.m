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
[qi, qdi,qddi]= fourier_f(test);


%% Condition number of regressor
for i=1:length(qi(1,:)) 
    W(1+(i-1)*gen.dof:gen.dof+(i-1)*gen.dof,:)=gen.condfun(qi(1,i),qi(2,i),qi(3,i),qdi(1,i),qdi(2,i),qdi(3,i),qddi(1,i),qddi(2,i),qddi(3,i));
 %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(i,1:3), qdf(i,1:3), acc(i,1:3)]);
end

fs.cond=cond(W);
%% plot
figure()
%title('joint positions')
plot(test.time,qi(1,:),test.time,qi(2,:),test.time,qi(3,:));
legend('1','2','3')

figure()
title('joint velocities')
plot(test.time,qdi(1,:),test.time,qdi(2,:),test.time,qdi(3,:));
legend('1','2','3')

end