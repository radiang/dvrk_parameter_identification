function [gen,traj]=new_brute_trajectory(gen,traj)
% 
% filename='new_6dof_inaxis_svd';
% savename= strcat(filename);
% loadname = strcat('../dynamics_derivation/data/',filename,'_Y.mat');
% load(loadname);

%% Options
tf=traj.tf;
ts=traj.ts;
par_num=length(gen.Par2);
dof_num = gen.dof;
point_num=traj.point_num;
iter = traj.iter;

%% Joint Limits
limit_min(1)=-1.54; %rad
limit_max(1)=1.54;
limit_min(2)=-0.84; %rad
limit_max(2)=0.84;  
limit_min(3)=0;     %m
limit_max(3)=0.24; 
limit_min(4)=-1.54; %rad
limit_max(4)=1.54;
limit_min(5)=-1.54; %rad
limit_max(5)=1.54;
limit_min(6)=-1.54; %rad
limit_max(6)=1.54;

%Maximum Velocity 
velocity_min(1)= -0.5 ;%rad/s
velocity_max(1)= 0.5 ;%rad/s
velocity_min(2)= -0.06;
velocity_max(2)= 0.06; %m/s

%Chosen Trajectories

options=10;
for i=1:6
C(i,:)=linspace(limit_min(i),limit_max(i),options); 
end

for i=1:6
Cd(i,:)=linspace(velocity_min(1),velocity_max(1),options);
    if i==3
        Cd(i,:)=linspace(velocity_min(2),velocity_max(2),options);
    end
end

B=permn(linspace(1,options,options),dof_num);

%Completely Random Trajectories
Cond=zeros(1,iter);
Condf=zeros(1,iter);


for j=1:iter

for dof=1:dof_num
array = [0];
array_v = [0];
for n = 1:point_num
    
temp = randi(options);

   
r(n) = C(dof,temp);
rd(n) = Cd(dof,temp);

array(n+1) = r(n);
array_v(n+1) = rd(n);
end
traj.save_array(dof,:,j)=array(:);
traj.save_arrayv(dof,:,j)=array_v(:);

[q(dof,:),qd(dof,:),qdd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end

W=zeros(2*length(q),par_num);

%Test Trajectory for Least squares solution
for i=1:length(q(1,:))

 W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=gen.condfun(q(1,i),q(2,i),q(3,i),qd(1,i),qd(2,i),qd(3,i),qdd(1,i),qdd(2,i),qdd(3,i)) ;
  
 %W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=subs(Ys2, transpose([Q(1:dof_num); Qd(1:dof_num) ;Qdd(1:dof_num)]),[q(1:dof_num,i)', qd(1:dof_num,i)', qdd(1:dof_num,i)']);

end



traj.Cond(j)=cond(W,2);
traj.Condf(j)=cond(W'*W,'fro');
W=zeros(2*length(q),par_num);
end 

[traj.brute_opt_cond ind]=min(traj.Cond(1:iter));
traj.brute_opt_pos =traj.save_array(:,:,ind);
traj.brute_opt_vel = traj.save_arrayv(:,:,ind);

% savename= strcat('data/',savename,'_trajectory2.mat');
% save(savename)

end

