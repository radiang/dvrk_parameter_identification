function compare_effort_simulated(gen,ident,test)
ident.tau = ident.tauf;
%% Compare with a different trajectory

ident = [];
load('compare2.mat')
ident = compare;


%% Least squares and weights least squares
for i=1:length(ident.qddi) 
    sim_tor_ls(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.ls_par2;
    sim_tor_wls(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.wls_par2;
    %sim_tor_sdp(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.sdp_par2;
end



%% Torque
figure()
N = min([length(ident.qddi), length(ident.tau)]);
error_ls = abs(ident.tau(1:N,:).' - sim_tor_ls(:,1:N));
error_wls =abs( ident.tau(1:N,:).' - sim_tor_wls(:,1:N));

for i = 1:gen.dof
subplot(gen.dof,1,i)
plot(test.time(1:N),ident.tauf(1:N,i),test.time(1:N),sim_tor_ls(i,1:N), test.time(1:N),sim_tor_wls(i,1:N));
hold on
%plot(test.time(1:N),sim_tor_sdp(i,1:N));
hold on
%plot(test.time(1:N), error_ls(i,1:N),test.time(1:N), error_wls(i,1:N))
hold off

titlename =sprintf('Joint %d',i);
title(titlename)
legend('Measured','LS','WLS','error_LS','error_WLS')

end





