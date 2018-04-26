function [Cond]=new_cond(x)

load('Ys2.mat');

ts=0.5;
tf=ts*(10-1);
t=0:ts:tf;
%r=length(Q)-1;

array(1,:) = [0 x(1) x(2) x(3) x(4) x(5)];
array(2,:) = [0 x(6) x(7) x(8) x(9) x(10)];
array(3,:) = [0 x(11) x(12) x(13) x(14) x(15)];
array(4,:) = [0 x(16) x(17) x(18) x(19) x(20)];
array(5,:) = [0 x(21) x(22) x(23) x(24) x(25)];
array(6,:) = [0 x(26) x(27) x(28) x(29) x(30)];

array_v(1,:) = [0 x(31) x(32) x(33) x(34) x(35)];
array_v(2,:) = [0 x(36) x(37) x(38) x(39) x(40)];
array_v(3,:) = [0 x(41) x(42) x(43) x(44) x(45)];
array_v(4,:) = [0 x(46) x(47) x(48) x(49) x(50)];
array_v(5,:) = [0 x(51) x(52) x(53) x(54) x(55)];
array_v(6,:) = [0 x(56) x(57) x(58) x(59) x(60)];



for dof=1:6
[Q(dof,:),Qd(dof,:),Qdd(dof,:),T]=Trajectory_f(array(i,:),array_v(i,:),tf,ts);
end

for i=1:length(Q(1,:))
   

W(1+(i-1)*6:6+(i-1)*6,:)=subs(Ys2, [q qd qdd],[Q(1:6,i)', Qd(1:6,i)', Qdd(1:6,i)']);

end


Cond = cond(W);


end