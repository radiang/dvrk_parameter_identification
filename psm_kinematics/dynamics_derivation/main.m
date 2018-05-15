clear all

%% Options 
traj.iter = 60;
traj.tf = 2;
traj.ts = 0.01;
traj.point_num=14;

traj.limit_pos=[1.4, 0.8, 0.23, 1.5, 1.5, 1.5];
traj.limit_vel=[2, 2, 0.4, 0.4, 0.4, 0.4];
traj.scale_p = 0.6;
traj.scale_v = 0.6;

%% Dynamics Derivation
[gen, dyn,map] = psm_dynamics_f();

%% Trajectory Optimization 
gen.condfun=matlabFunction(gen.Ys2);

[gen,traj]=new_brute_trajectory(gen,traj);

%% Optimal Trajectory
[gen,traj]=new_optimization(gen,traj);

% Visualize data
for i=1:gen.dof
   [opt(i,:),optd(i,:),optdd(i,:)]=Trajectory_f(traj.opt_pos(i,:),traj.opt_vel(i,:),traj.tf,traj.ts,1);
end

savename=strcat('data/',gen.filename,'/_optimized.mat');
save(savename);



%% Parameter Identification
gen.csvfilename='test1';
ident.window = 12; 
ident.a=1;

[gen,traj,ident]=new_par_ident(gen,traj,ident,1);

%% SDP OLS
[gen] = SDP_OLS(gen,ident,dyn,map);

%Test Inverse_map

%% Test Parameter Identification force 
eff.traj_p = [0 0.2 0 ; 0 0.2 0; 0 0.1 0.15];
eff.traj_v = zeros(3);
eff.traj_a = zeros(3);
eff.tf = 2;
eff.ts = 0.01;  
[eff] = make_effort(gen,eff);

%% Save
savename=strcat('data/',gen.filename,'/',gen.csvfilename,'_results.mat');
save(savename);

%% Compare effort
%Compare effort of open loop effort to desired trajectory using identified
%parameters 
[eff] = compare_effort(gen,eff);

%%  Fourier Trajectory Optimization
%[fs,gen]=fourier_trajectory(gen,ident,traj);
gen.fourfilename = 'fourier_test';

[fs,gen]=fourier_trajectory_run(gen,ident,traj);

[fs,gen] = check_fourier(gen,ident,traj,fs);

savename=strcat('data/',gen.filename,'/',gen.fourfilename,'.mat');
save(savename);


%% Fourier Trajectory Identification
gen.fstestname = 'test1';
[gen,fs,ident]=fs_par_ident(gen,fs,ident,1);

%% Make force Controllers 
[gen,traj,dyn,ctrl,init]=impedance_control_run(gen,traj,dyn);


