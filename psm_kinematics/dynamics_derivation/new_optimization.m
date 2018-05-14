function [gen,traj]=new_optimization(gen,traj)
% filename = 'new_6dof_inaxis_svd';
% loadname = strcat('data/',filename,'_trajectory.mat');
% load(loadname)

%global Y dof_max n
Y=gen.Ys2;

n = length(traj.brute_opt_pos)-1; %Data points
dof_max=gen.dof;
data_max= n*dof_max;
scale_p = traj.scale_p;
scale_v = traj.scale_v;

%% limits

for i = 1:length(traj.limit_pos)
limit_min(i)=-traj.limit_pos(i); %rad
limit_max(i)=traj.limit_pos(i);

if i == 3
    limit_min(i)=0;
end
       
velocity_min(i)= -traj.limit_vel(i) ;%rad/s
velocity_max(i)= traj.limit_vel(i) ;%rad/s
end


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

lb(data_max+(dof-1)*n+data)=[velocity_min(dof)];
ub(data_max+(dof-1)*n+data)=[velocity_max(dof)];
if(dof==3)
    lb(data_max+(dof-1)*n+data)=[velocity_min(dof)];
    ub(data_max+(dof-1)*n+data)=[velocity_max(dof)];
end
end
end


lb = scale_p*lb;
ub = scale_v*ub;

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

x0 = reshape(traj.brute_opt_pos(:,2:end)',1,[]);
xv0 = reshape(traj.brute_opt_vel(:,2:end)',1,[]);

%% Run fmincon
size = length(gen.Par2);
fun = @(z) new_cond(z,gen.Ys2,n,dof_max,gen.condfun,size,traj.tf,traj.ts);

options = optimoptions('fmincon','MaxIterations',3000,'MaxFunctionEvaluations',10000);
[vars, traj.opt_cond]=fmincon(fun,[x0, xv0],A,b,Ae,be,lb,ub,[],options);


%% Save

traj_p= reshape(vars(1:end/2),[],dof_max)';
traj_v= reshape(vars((end/2+1):end),[],dof_max)';

traj.opt_pos = [zeros(dof_max,1),traj_p,zeros(dof_max,1)];
traj.opt_vel = [zeros(dof_max,1),traj_v,zeros(dof_max,1)];


%% Visuallize data



% savename = strcat('data/',filename,'_optimized.mat');
% save(savename)
% %g=10;
%fun = @(x) try_cond(x,g);

%[vars, Fin]=fmincon(fun,[.1 .5 .3400 .123],A,b,Ae,be,[-1 -1 -1 -1],[1 1 1 1]);
%save('Optimized_Brute_2.mat')

%% Make csv
for dof=1:dof_max
[ass(dof,:),ass(dof+dof_max,:),ass(dof+dof_max*2,:),T]=Trajectory_f(traj.opt_pos(dof,:),traj.opt_vel(dof,:),traj.tf,traj.ts,0);
end

 csvname=strcat('data/',gen.filename,'/traj2.csv');
 csvwrite(csvname,ass);

end
