function [ctrl] =pos_check(gen,ctrl,traj)
%% Check for Positive Semidefiniteness
 N =1000;
 
Q = zeros(gen.dof,1);
Qd = zeros(gen.dof,1);
Qdd = zeros(gen.dof,1);

for i = 1:gen.dof
    for j = 1:N
     Q(i,j)=traj.limit_pos(i)*rand();
     Qd(i,j) = traj.limit_vel(i)*rand();
     Qdd(i,j) = 0;
    end
end
tf = 2;
ts = 0.1;

for i=1:gen.dof
[qi(i,:),qdi(i,:),qddi(i,:),T]= Trajectory_quintic(Q(i,:),Qd(i,:),Qdd(i,:),tf,ts,0,0);
end


 check_M = zeros(gen.dof,gen.dof,length(qi));
 p = zeros(1,length(qi));
 
 tempfun =matlabFunction(ctrl.Mt3);
 for i =1 :length(qi)
     tic
    ctrl.check_M(:,:,i) = tempfun(qi(2,i),qi(3,i));
    
   %check_M(:,:,i) = subs(ctrl.Mt3,gen.q(1:gen.dof),[fs.qi(1,i), fs.qi(2,i), fs.qi(3,i)]);
   [~,p(i)]=chol(ctrl.check_M(:,:,1));
      toc
 end
 ctrl.p = p;
 ctrl.sizeM = size(ctrl.check_M);
 
 x=0;
end
