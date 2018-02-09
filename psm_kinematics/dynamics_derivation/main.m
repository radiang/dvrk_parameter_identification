clear all
%% Dynamics Derivation
[Y, tau, Par] = psm_dynamics_f();

%% Lumping Parameters 
% Note: Passing the Matrix Y uses subs function, which is slow. 
%finding the linear combinations
[Ys1, Ys2, Par1, Par2]=lumping_parameters(Y,Par);

 %% Trajectory Optimization 

[x, v, cond_save]=traj_opt_rand(Ys2,size(Ys2,2));


%% Show the Results
disp('Resulting Regressor Matrix: ')
Ys2

disp('Resulting Identifiable Parameters: ')
Par2
