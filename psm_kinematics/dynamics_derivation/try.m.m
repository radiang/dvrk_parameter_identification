
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
save_array(dof,:,j)=array(:);
save_arrayv(dof,:,j)=array_v(:);

[Q(dof,:),Qd(dof,:),Qdd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end



%Enlarge the matrix with random numbers
Ww=zeros(dof_num*length(Q),length(Par2));
xx=zeros(1,dof_num);
xxd = zeros(1,dof_num);
xxdd = zeros(1,dof_num);

for i=1:length(Q(1,:))
    for j = 1:dof_num
        xx(1,j)=Q(j,i);
        xxd(1,j)=Qd(j,i);
        xxdd(1,j)=Qdd(j,i);
    end

Ww(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=double(subs(Ys2,[q(1:dof_num), qd(1:dof_num), qdd(1:dof_num)],[xx(1,:),xxd(1,:),xxdd(1,:)]));

end