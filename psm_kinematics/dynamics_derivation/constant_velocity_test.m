function constant_velocity_test(gen)
close all
offset = 0.005;
limit_min = 0+offset;
limit_max = 0.22-offset;

rate = 200;
ts = 1/rate;
period = 1; %s

vel_max = 0.3;
data_point = 16;
speeds = linspace(0, vel_max, data_point); %rad/s ;

speeds(1) = [];

begin = 0.03;
sec_limit = 1;

for i = 1:length(speeds)
    period = (limit_max-begin)/speeds(i);
    
    if period> sec_limit
       period = sec_limit; 
    end
    
    %Ramp Up
    [x(i,:),v,ac,T] =  Trajectory_quintic([limit_min, begin],[0 speeds(i)], [0,0], 0.2 ,ts,0,0);
    q(i,1) = begin;

    %Constant Velocity
    for j = 1:round(period*rate)-1
         q(i,j+1) = q(i,j) + speeds(i)*ts;
    end
    %Ramp down 
    [x_end(i,:),v,ac,T] =  Trajectory_quintic([max(q(i,:)), limit_min],[0 0], [0,0], 2 ,ts,0,0);
end

q = [x,q, x_end];

q(q==0)=nan;
traj = reshape(q.',1,[]);
traj_time = 1:length(traj);
traj_time = traj_time.*ts;

figure()
plot(traj_time,traj);

csvname=strcat('data/',gen.filename,'/frtest.csv');
csvwrite(csvname,q);


q_neg = -q + (limit_max+offset);
traj_neg = reshape(q_neg.',1,[]);

figure()
plot(traj_time,traj_neg);
csvname=strcat('data/',gen.filename,'/frtest_neg.csv');
csvwrite(csvname,q_neg);


%% Force Ramp test

f(:,1) = linspace(limit_min+0.015,limit_max,length(speeds));
f(:,2) = linspace(limit_min+0.015,limit_max,length(speeds));

f = reshape(f.',[],1);

max_F = 1;
num= 500;

for i = 1:length(f)
    if rem(i,2)==0
        force(i,:) = linspace(0,max_F,num);
    else
        force(i,:) = - linspace(0,max_F,num);
    end
    
end

f_vec = [f,force];

x = 0;

csvname=strcat('data/',gen.filename,'/brtest.csv');
csvwrite(csvname,f_vec);

end