function [eff] = compare_effort_simulated(gen,ident,test,fs)

for i=1:length(fs.qi) 
    sim_tor_ls(:,i)=gen.condfun(fs.qi(1,i),fs.qi(2,i),fs.qi(3,i),fs.qdi(1,i),fs.qdi(2,i),fs.qdi(3,i),fs.qddi(1,i),fs.qddi(2,i),fs.qddi(3,i))*gen.ls_par2;
    sim_tor_wls(:,i)=gen.condfun(fs.qi(1,i),fs.qi(2,i),fs.qi(3,i),fs.qdi(1,i),fs.qdi(2,i),fs.qdi(3,i),fs.qddi(1,i),fs.qddi(2,i),fs.qddi(3,i))*gen.wls_par2;
end



%% Torque
figure()
N = min([length(fs.qi), length(ident.tau)]);
error_ls = abs(ident.tau(1:N,:).' - sim_tor_ls(:,1:N));
error_wls =abs( ident.tau(1:N,:).' - sim_tor_wls(:,1:N));

for i = 1:gen.dof
subplot(gen.dof,1,i)
plot(test.time(1:N),ident.tau(1:N,i),test.time(1:N),sim_tor_ls(i,1:N), test.time(1:N),sim_tor_wls(i,1:N),...
    test.time(1:N), error_ls(i,1:N),test.time(1:N), error_wls(i,1:N));

titlename =sprintf('Joint %d',i);
title(titlename)
legend('Measured','LS','WLS','error_LS','error_WLS')

end





