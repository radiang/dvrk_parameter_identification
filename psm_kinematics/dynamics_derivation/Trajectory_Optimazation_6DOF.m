clear all

par_num=42;
W_rows=6;
%nd=(W_rows+1)*2*par_num;

t=2.5;
ts=0.5;
data_points=10;
nd=data_points*W_rows; 

Ai=[];
b=[];
load('A.mat');
A=double(A);
be=zeros(length(A(:,1)),1);
%Initial Guess
x0=0.6*rand(1,180);

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
velocity_min(1)= -0.5 ;%rad/s
velocity_max(1)= 0.5 ;%rad/s
velocity_min(2)= -0.06;
velocity_max(2)= 0.06; %m/s

for i=1:data_points
lb(18*(i-1)+1)=limit_min(1);
lb(18*(i-1)+2)=limit_min(2);
lb(18*(i-1)+3)=limit_min(3);
lb(18*(i-1)+4)=limit_min(4);
lb(18*(i-1)+5)=limit_min(5);
lb(18*(i-1)+6)=limit_min(6);

lb(18*(i-1)+7)=velocity_min(1);
lb(18*(i-1)+8)=velocity_min(1);
lb(18*(i-1)+9)=velocity_min(2);
lb(18*(i-1)+10)=velocity_min(1);
lb(18*(i-1)+11)=velocity_min(1);
lb(18*(i-1)+12)=velocity_min(1);

lb(18*(i-1)+13)=0;
lb(18*(i-1)+14)=0;
lb(18*(i-1)+15)=0;
lb(18*(i-1)+16)=0;
lb(18*(i-1)+17)=0;
lb(18*(i-1)+18)=0;

ub(18*(i-1)+1)=limit_max(1);
ub(18*(i-1)+2)=limit_max(2);
ub(18*(i-1)+3)=limit_max(3);
ub(18*(i-1)+4)=limit_max(4);
ub(18*(i-1)+5)=limit_max(5);
ub(18*(i-1)+6)=limit_max(6);

ub(18*(i-1)+7)=velocity_max(1);
ub(18*(i-1)+8)=velocity_max(1);
ub(18*(i-1)+9)=velocity_max(2);
ub(18*(i-1)+10)=velocity_max(1);
ub(18*(i-1)+11)=velocity_max(1);
ub(18*(i-1)+12)=velocity_max(1);

ub(18*(i-1)+13)=0;
ub(18*(i-1)+14)=0;
ub(18*(i-1)+15)=0;
ub(18*(i-1)+16)=0;
ub(18*(i-1)+17)=0;
ub(18*(i-1)+18)=0;
end



[x ,Fval]=fmincon(@objective_cond_6dof,x0,Ai,b,A,be,lb,ub);