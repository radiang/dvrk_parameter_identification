clear all 
close all

filename = 'new_3dof_inaxis_svd';
loadname = strcat('data/',filename,'_trajectory.mat');
load(loadname)

%global Y dof_max n
Y=Ys2;

n = length(best_pos)-1; %Data points
dof_max=dof_num;
data_max= n*dof_max;

%% limits
limit_min(1)=-1.541; %rad
limit_max(1)=1.541;
limit_min(2)=-0.841; %rad
limit_max(2)=0.841;  
limit_min(3)=0.03;     %m
limit_max(3)=0.241; 
limit_min(4)=-1.541; %rad
limit_max(4)=1.541;
limit_min(5)=-1.541; %rad
limit_max(5)=1.541;
limit_min(6)=-1.541; %rad
limit_max(6)=1.541;

%Maximum Velocity 
velocity_min(1)= -0.4 ;%rad/s
velocity_max(1)= 0.4 ;%rad/s
velocity_min(2)= -0.101;
velocity_max(2)= 0.101; %m/s

%Constraints
A=[];
b=[];
Ae=[];
be=[];

%Bounds
for dof = 1:dof_max
for data = 1:n

lb((dof-1)*n+data)=[limit_min(dof)];
ub((dof-1)*n+data)=[limit_max(dof)];
end
end

for dof = 1:dof_max
for data = 1:n

lb(data_max+(dof-1)*n+data)=[velocity_min(1)];
ub(data_max+(dof-1)*n+data)=[velocity_max(1)];
if(dof==3)
    lb(data_max+(dof-1)*n+data)=[velocity_min(2)];
    ub(data_max+(dof-1)*n+data)=[velocity_max(2)];
end
end
end

scale = 0.8;
lb = scale*lb;
ub = scale*ub;

%% Initial Guess

%Chosen Trajectories
% options=7;
% for i=1:6
% C(i,:)=linspace(limit_min(i),limit_max(i),options); 
% end
% 
% for i=1:6
% Cd(i,:)=linspace(velocity_min(1),velocity_max(1),options);
%     if i==3
%         Cd(i,:)=linspace(velocity_min(2),velocity_max(2),options);
%     end
% end



% for dof = 1:dof_max
% for data = 1:n
% 
%     
% x0((dof-1)*n+data)=C(dof,randi(options));
% xv0((dof-1)*n+data)=Cd(dof,randi(options));
% 
% end
% end

x0 = reshape(best_pos(:,2:end)',1,[]);
xv0 = reshape(best_vel(:,2:end)',1,[]);

%% Run fmincon
fun = @(z) new_cond(z,Ys2,n,dof_max,transpose(Q),transpose(Qd),transpose(Qdd));

options = optimoptions('fmincon','MaxIterations',6000);
[vars, Fin]=fmincon(fun,[x0, xv0],A,b,Ae,be,lb,ub,[],options);


%% Save
tf = 2;
ts = 0.01;

traj_p= reshape(vars(1:end/2),[],dof_max)';
traj_v= reshape(vars((end/2+1):end),[],dof_max)';

traj_p = [zeros(dof_max,1),traj_p,zeros(dof_max,1)];
traj_v = [zeros(dof_max,1),traj_v,zeros(dof_max,1)];

for i=1:dof_num
   [opt(i,:),optd(i,:),optdd(i,:)]=Trajectory_f(traj_p(i,:),traj_v(i,:),tf,ts,1);
end


savename = strcat('data/',filename,'_optimized.mat');
save(savename)
%g=10;
%fun = @(x) try_cond(x,g);

%[vars, Fin]=fmincon(fun,[.1 .5 .3400 .123],A,b,Ae,be,[-1 -1 -1 -1],[1 1 1 1]);
%save('Optimized_Brute_2.mat')

%% Make csv
for dof=1:dof_num
[traj(dof,:),traj(dof+dof_num,:),traj(dof+dof_num*2,:),T]=Trajectory_f(traj_p(dof,:),traj_v(dof,:),tf,ts,0);
end

csvname=strcat('data/',filename,'_traj.csv');
csvwrite(csvname,traj);
