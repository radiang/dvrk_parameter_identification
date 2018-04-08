function [Cond]=new_cond(z,Y,n,dof_max,q,qd,qdd)

Ys2=Y;

ts=0.1;
tf=2;

for j = 1:dof_max
for t=1:n
a = sprintf('z(%d)',(j-1)*n+t);
b = sprintf('array(%d,%d)',[j, t+1]);
eval(strcat(b,'=',a));

a2 = sprintf('z(%d)',dof_max*n+(j-1)*n+t);
b2 = sprintf('array_v(%d,%d)',[j,t+1]);
eval(strcat(b2,'=',a2));

end 
end


% array(1,:) = [0 x(1) x(2) x(3) x(4) x(5)];
% 
% array(2,:) = [0 x(6) x(7) x(8) x(9) x(10)];
% array(3,:) = [0 x(11) x(12) x(13) x(14) x(15)];
% % array(4,:) = [0 x(16) x(17) x(18) x(19) x(20)];
% % array(5,:) = [0 x(21) x(22) x(23) x(24) x(25)];
% % array(6,:) = [0 x(26) x(27) x(28) x(29) x(30)];
% 
% array_v(1,:) = [0 x(31) x(32) x(33) x(34) x(35)];
% array_v(2,:) = [0 x(36) x(37) x(38) x(39) x(40)];
% array_v(3,:) = [0 x(41) x(42) x(43) x(44) x(45)];
% 
% array_v(1,:) = [0 x(31) x(32) x(33) x(34) x(35)];
% array_v(2,:) = [0 x(36) x(37) x(38) x(39) x(40)];
% array_v(3,:) = [0 x(41) x(42) x(43) x(44) x(45)];
% array_v(4,:) = [0 x(46) x(47) x(48) x(49) x(50)];
% array_v(5,:) = [0 x(51) x(52) x(53) x(54) x(55)];
% array_v(6,:) = [0 x(56) x(57) x(58) x(59) x(60)];

for dof=1:dof_max
[Q(dof,:),Qd(dof,:),Qdd(dof,:),T]=Trajectory_f(array(dof,:),array_v(dof,:),tf,ts);
end

for i=1:length(Q(1,:))
   

W(1+(i-1)*dof_max:dof_max+(i-1)*dof_max,:)=subs(Ys2, [q(1:dof_max) qd(1:dof_max) qdd(1:dof_max)],[transpose(Q(1:dof_max,i)), transpose(Qd(1:dof_max,i)), transpose(Qdd(1:dof_max,i))]);

end


Cond = double(cond(W));


end