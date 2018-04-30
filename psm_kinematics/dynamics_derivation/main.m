clear all

%% Options 
traj.iter = 50;
traj.tf = 2;
traj.ts = 0.01;
traj.point_num=12;

traj.limit_pos=[1.5, 0.841, 0.03, 1.5, 1.5, 1.5];
traj.limit_vel=[0.4, 0.1, 0.4, 0.4, 0.4, 0.4];
traj.scale = 0.8;

%% Dynamics Derivation
[gen, dyn] = psm_dynamics_f();

%% Trajectory Optimization 
gen.condfun=matlabFunction(gen.Ys2);
[gen,traj]=new_brute_trajectory(gen,traj);

%% Optimal Trajectory
[gen,traj]=new_optimization(gen,traj);
save('temp1.mat')

%% Parameter Identification
gen.csvfilename='PID_data_0.9';
ident.window = 12; 
ident.a=1;

[gen,traj,ident]=new_par_ident(gen,traj,ident,1);

%% Save
savename=strcat('data/',gen.filename,'/',gen.csvfilename,'_results.mat');
save(savename);


%% Test Parameter Identification force 
eff.traj_p = [0 0.5 0 ; 0 0.3 0; 0 0.1 0.15];
eff.traj_v = zeros(3);
eff.traj_a = zeros(3);
eff.tf = 2;
eff.ts = 0.1;  
[eff] = make_effort(gen,eff);

%%  Fourier Trajectory Optimization
[fs,gen]=fourier_trajectory(gen,ident,traj);

%% Make force Controllers 
[gen,traj,dyn,ctrl]=impedance_control_run(gen,traj,dyn);
