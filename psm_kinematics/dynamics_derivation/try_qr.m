%clear all

dof_num=6;

tf=2;
ts=0.1;
g=9.8;
par_num=length(Par);

point_num=5;

%Joint Limits
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
velocity_min(1)= -0.7 ;%rad/s
velocity_max(1)= 0.7 ;%rad/s
velocity_min(2)= -0.06;
velocity_max(2)= 0.06; %m/s

%Chosen Trajectories
options=7;
for i=1:6
op(i,:)=linspace(limit_min(i),limit_max(i),options); 
end

for i=1:6
opd(i,:)=linspace(velocity_min(1),velocity_max(1),options);
    if i==3
        opd(i,:)=linspace(velocity_min(2),velocity_max(2),options);
    end
end


for dof=1:dof_num
array = [0];
array_v = [0];
for n = 1:point_num
    
temp = randi(options);
temp2 = randi(options);
   
r(n) = op(dof,temp);
rd(n) = opd(dof,temp2);

array(n+1) = r(n);
array_v(n+1) = rd(n);
end
%save_array(dof,:,j)=array(:);
%save_arrayv(dof,:,j)=array_v(:);

[Qe(dof,:),Qed(dof,:),Qedd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end



%Enlarge the matrix with random numbers
W=zeros(dof_num*length(Qe),par_num);
xx=zeros(1,dof_num);
xxd = zeros(1,dof_num);
xxdd = zeros(1,dof_num);

for i=1:length(Qe(1,:))
    for j = 1:dof_num
        xx(1,j)=Qe(j,i);
        xxd(1,j)=Qed(j,i);
        xxdd(1,j)=Qedd(j,i);
    end

W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=double(subs(Y,[q(1:dof_num), qd(1:dof_num), qdd(1:dof_num)],[xx(1,:),xxd(1,:),xxdd(1,:)]));

end

[Q,R,P] = qr(W);

c = size(W,2);
b = rank(W);

R1 =  R(1:b,1:b);
R2 = R(1:b,b+1:end);

temp=P*transpose(Par);
X1 = temp(1:b);
X2 = temp(b+1:end);

XB1 = X1+round(inv(R1)*R2,5)*X2;

save1=inv(R1)*R2;

W(:,:)= [];
%%
for dof=1:dof_num
array = [0];
array_v = [0];
for n = 1:point_num
    
temp = randi(options);
temp2 = randi(options);
   
r(n) = op(dof,temp);
rd(n) = opd(dof,temp2);

array(n+1) = r(n);
array_v(n+1) = rd(n);
end
save_array(dof,:,j)=array(:);
save_arrayv(dof,:,j)=array_v(:);

[Qe(dof,:),Qed(dof,:),Qedd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end



%Enlarge the matrix with random numbers
W=zeros(dof_num*length(Qe),par_num);
xx=zeros(1,dof_num);
xxd = zeros(1,dof_num);
xxdd = zeros(1,dof_num);

for i=1:length(Qe(1,:))
    for j = 1:dof_num
        xx(1,j)=Qe(j,i);
        xxd(1,j)=Qed(j,i);
        xxdd(1,j)=Qedd(j,i);
    end

W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=double(subs(Y,[q(1:dof_num), qd(1:dof_num), qdd(1:dof_num)],[xx(1,:),xxd(1,:),xxdd(1,:)]));

end

[Q,R,P] = qr(W);

c = size(W,2);
b = rank(W);

R1 =  R(1:b,1:b);
R2 = R(1:b,b+1:end);

temp=P*transpose(Par);
X1 = temp(1:b);
X2 = temp(b+1:end);

XB1_2 = X1+round(inv(R1)*R2,6)*X2;

save2=inv(R1)*R2;


%%
for dof=1:dof_num
array = [0];
array_v = [0];
for n = 1:point_num
    
temp = randi(options);
temp2 = randi(options);
   
r(n) = op(dof,temp);
rd(n) = opd(dof,temp2);

array(n+1) = r(n);
array_v(n+1) = rd(n);
end
save_array(dof,:,j)=array(:);
save_arrayv(dof,:,j)=array_v(:);

[Qe(dof,:),Qed(dof,:),Qedd(dof,:),T]=Trajectory_f(array,array_v,tf,ts);

end



%Enlarge the matrix with random numbers
W=zeros(dof_num*length(Qe),par_num);
xx=zeros(1,dof_num);
xxd = zeros(1,dof_num);
xxdd = zeros(1,dof_num);

for i=1:length(Qe(1,:))
    for j = 1:dof_num
        xx(1,j)=Qe(j,i);
        xxd(1,j)=Qed(j,i);
        xxdd(1,j)=Qedd(j,i);
    end

W(1+(i-1)*dof_num:dof_num+(i-1)*dof_num,:)=double(subs(Y,[q(1:dof_num), qd(1:dof_num), qdd(1:dof_num)],[xx(1,:),xxd(1,:),xxdd(1,:)]));

end

[Q,R,P] = qr(W);

c = size(W,2);
b = rank(W);

R1 =  R(1:b,1:b);
R2 = R(1:b,b+1:end);

temp=P*transpose(Par);
X1 = temp(1:b);
X2 = temp(b+1:end);

XB1_3 = X1+round(inv(R1)*R2,6)*X2;

save3=inv(R1)*R2;



vpa(XB1,2)
vpa(XB1_2,2)
vpa(XB1_3,2)