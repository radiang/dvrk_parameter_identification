clear all
close all

filename = '3dof_inplanepitch_svd';
loadname = strcat('data/',filename,'_results_twice.mat');
load(loadname);

%% Fourier Expansion

F.a=sym('a%d',[3]);
F.b=sym('a%d',[3]);