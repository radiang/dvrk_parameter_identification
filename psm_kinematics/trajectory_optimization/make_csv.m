clear all

for dof=1:dof_num
[traj(dof,:),traj(dof+dof_num,:),traj(dof+dof_num*2,:),T]=Trajectory_f(traj_p(dof,:),traj_v(dof,:),tf,ts,1);
end

csvname=strcat('data/',filename,'_traj.csv');
csvwrite(csvname,traj);
