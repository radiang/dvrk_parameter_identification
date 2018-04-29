function [eff] = make_effort(gen, eff)

close all
for i = 1:gen.dof
[joint(i,:), jointd(i,:), jointdd(i,:), T] = Trajectory_quintic(eff.traj_p(i,:),eff.traj_v(i,:),eff.traj_a(i,:),eff.tf,eff.ts,1);
end


for j = 1:length(joint)

    for z = 1:gen.dof
       q(z)   = joint(z,j);
       qd(z)  = jointd(z,j);
       qdd(z) = jointdd(z,j);
    end
    
eff.u(1:gen.dof,j) = gen.condfun(q(1),q(2),q(3),qd(1),qd(2),qd(3),qdd(1),qdd(2),qdd(3))*gen.par_num;

end

csvname = strcat('data/',gen.filename,'/', gen.csvfilename,'_test_effort.csv');
csvwrite(csvname,eff.u.');

end