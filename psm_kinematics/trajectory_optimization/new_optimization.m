clear all 
close all

data_max = 5;
dof_max = 6;
n= data_max*dof_max;
%Initial Guess
x0=0.1*ones(1,n);
xv0=0.05*ones(1,n);

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
velocity_min(2)= -0.10;
velocity_max(2)= 0.10; %m/s

%Constraints
A=[];
b=[];
Ae=[];
be=[];

%Bounds
for data = 1:data_max
for dof = 1:dof_max
lb((dof-1)*6+data)=[limit_min(dof)];
ub((dof-1)*6+data)=[limit_max(dof)];
end
end

for data = 1:data_max
for dof = 1:dof_max
lb(n+(dof-1)*6+data)=[velocity_min(1)];
ub(n+(dof-1)*6+data)=[velocity_max(1)];
if(dof==3)
    lb(n+(dof-1)*6+data)=[velocity_min(2)];
    ub(n+(dof-1)*6+data)=[velocity_max(2)];
end
end
end

[vars Fin]=fmincon(f,[x0 xv0,A,b,Ae,be,lb,ub);

