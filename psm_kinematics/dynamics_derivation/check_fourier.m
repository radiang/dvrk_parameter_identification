function [fs,gen]=check_fourier(gen,ident,traj,fs);

test.v= reshape(fs.vars,gen.dof,[]);
test.dof = gen.dof;
test.time = fs.time;
test.N=fs.Nl;
test.w = fs.w;
%% Input
[qi, qdi,qddi]= fourier_f(test);


%% Condition number of regressor


%% plot
figure()
plot(test.time,qi(1,:),test.time,qi(2,:),test.time,qi(3,:));
legend('1','2','3')

end