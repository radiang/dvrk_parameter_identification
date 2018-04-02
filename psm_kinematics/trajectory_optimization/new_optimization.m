clear all 
close all

load('Brute_2.mat')

data_max = 5;
dof_max = 6;
n= data_max*dof_max;

%% limits
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
for dof = 1:dof_max
for data = 1:data_max

lb((dof-1)*data_max+data)=[limit_min(dof)];
ub((dof-1)*data_max+data)=[limit_max(dof)];
end
end

for dof = 1:dof_max
for data = 1:data_max

lb(n+(dof-1)*data_max+data)=[velocity_min(1)];
ub(n+(dof-1)*data_max+data)=[velocity_max(1)];
if(dof==3)
    lb(n+(dof-1)*data_max+data)=[velocity_min(2)];
    ub(n+(dof-1)*data_max+data)=[velocity_max(2)];
end
end
end

%% Initial Guess

%Chosen Trajectories
% options=7;
% for i=1:6
% C(i,:)=linspace(limit_min(i),limit_max(i),options); 
% end
% 
% for i=1:6
% Cd(i,:)=linspace(velocity_min(1),velocity_max(1),options);
%     if i==3
%         Cd(i,:)=linspace(velocity_min(2),velocity_max(2),options);
%     end
% end
% 
% 
% 
% for dof = 1:dof_max
% for data = 1:data_max
% 
%     
% x0((dof-1)*data_max+data)=C(dof,randi(options));
% xv0((dof-1)*data_max+data)=Cd(dof,randi(options));
% 
% end
% end

x0 = reshape(good(:,2:end)',1,[]);
xv0 = reshape(goodv(:,2:end)',1,[]);

%% Run fmincon
[vars, Fin]=fmincon(@new_cond,[x0 xv0],A,b,Ae,be,lb,ub);

save('Optimized_Brute_2.mat')
