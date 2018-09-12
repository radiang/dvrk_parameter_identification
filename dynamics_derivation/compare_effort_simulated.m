function [test]= compare_effort_simulated(gen,ident,test)
ident.tau = ident.tauf;
%% Compare with a different trajectory

% ident = [];
% loadname = strcat('data/',gen.filename,'/','fourier_test5','_compare.mat');
% load(loadname);
% ident = compare;

%% Least squares and weights least squares
for i=1:length(ident.qddi) 
    sim_tor_ls(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.ls_par2;
    sim_tor_wls(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.wls_par2;
    sim_tor_sdp(:,i)=gen.condfun(ident.qi(1,i),ident.qi(2,i),ident.qi(3,i),ident.qdi(1,i),ident.qdi(2,i),ident.qdi(3,i),ident.qddi(1,i),ident.qddi(2,i),ident.qddi(3,i))*gen.sdp_par2;
end

%% Settings
N = min([length(ident.qddi), length(ident.tau)]);
N = round(N/2);

error_ls = abs(ident.tauf(1:N,:).' - sim_tor_ls(:,1:N));
error_wls =abs( ident.tauf(1:N,:).' - sim_tor_wls(:,1:N));
error_sdp =abs( ident.tauf(1:N,:).' - sim_tor_sdp(:,1:N));

%% Torque


figure()
for i = 1:gen.dof
subplot(gen.dof,1,i)
plot(test.time(1:N),ident.tauf(1:N,i),test.time(1:N),sim_tor_ls(i,1:N), test.time(1:N),sim_tor_wls(i,1:N));
hold on
plot(test.time(1:N),sim_tor_sdp(i,1:N));

hold off
ylabel('Torque [Nm]')
titlename =sprintf('Measured vs Computed Trajectories of Joint %d',i);
title(titlename)
legend('Measured','LS','WLS', 'SDP')
if(i==3)
xlabel('Time [s]')
ylabel('Force [N]')
end
end

%% Errors
figure()
for i = 1:gen.dof
subplot(gen.dof,1,i)
plot(test.time(1:N), error_ls(i,1:N),test.time(1:N), error_wls(i,1:N))
hold on
plot(test.time(1:N), error_sdp(i,1:N))
hold off

ylabel('Torque Error [Nm]')
titlename =sprintf('Measured vs Computed Errors of Joint %d',i);
title(titlename)
legend('error LS','error WLS','error SDP')
if(i==3)
xlabel('Time [s]')
ylabel('Force Error [N]')
end
end
%% Accumulative error
 
test.total_ls = sum(error_ls.');
test.total2_ls = sum(test.total_ls);

test.total_wls = sum(error_wls.');
test.total2_wls = sum(test.total_wls);

test.total_sdp = sum(error_sdp.');
test.total2_sdp = sum(test.total_sdp);

disp('total ls errors: ');
test.total_ls
disp('total wls errors: ');
test.total_wls

end


