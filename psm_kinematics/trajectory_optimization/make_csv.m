clear all

for dof=1:dof_num
[traj(dof,:),traj(dof+dof_num,:),traj(dof+dof_num*2,:),T]=Trajectory_f(best_pos(dof,:),best_vel(dof,:),tf,ts);
end

csvname=strcat('data/',filename,'_traj.csv');
csvwrite(csvname,traj);
