function constant_velocity_test(gen)
close all
offset = 0.005;
limit_min = 0+offset;
limit_max = 0.22-offset;

rate = 200;
ts = 1/rate;
%%period = 1; %s

vel_max = 0.3;
data_point = 16;
speeds = linspace(0, vel_max, data_point); %rad/s ;

speeds(1) = [];

begin = 0.04;
sec_limit = 0.8;

spd_inc = 60;

traj = [];
trajv = [];

for i = 1:length(speeds)
    
    %Ramp up in velocity
    ramp = linspace(0,speeds(i),spd_inc+1);
    
    q(1) = limit_min;
    qv(1) = 0;
    ramp(1) = [];
%     if period> sec_limit -spd_inc/rate
%        period = sec_limit -spd_inc/rate; 
%     end
     for j = 1:(spd_inc)
         q(j+1) = q(j) + ramp(j)*ts;
         qv(j+1) = ramp(j);
     end
     
    period = (limit_max-q(end))/speeds(i);
    
    for j = spd_inc+1:(round(period*rate)+spd_inc+1)
    
         q(j+1) = q(j) + speeds(i)*ts;
         qv(j+1) = speeds(i);
    end
    q(end)
    %Ramp down 
    [x_end(1,:),v_end,ac,T] =  Trajectory_quintic([max(q), limit_min],[0 0], [0,0], 2 ,ts,0,0);
    
    traj = [traj, q, x_end];
    trajv = [ trajv, qv, v_end];
    q = [];
    x_end = [];
    qv = [];
    v_end = [];
    period = [];
    ramp = [];
    
end

% q = [q, x_end];
% qv = [qv, v_end];
% 
% q(q==0)=nan;
% traj = reshape(q.',1,[]);
traj_time = 1:length(traj);
traj_time = traj_time.*ts;

figure()
plot(traj_time,traj);

figure()
plot(traj_time,trajv);
title('velocities')

csvname=strcat('data/',gen.filename,'/frtest.csv');
csvwrite(csvname,traj);

traj_neg = -traj+limit_max;

figure()
plot(traj_time,traj_neg);
csvname=strcat('data/',gen.filename,'/frtest_neg.csv');
csvwrite(csvname,traj_neg);


%% Force Ramp test

f(:,1) = linspace(limit_min+0.015,limit_max,length(speeds));
f(:,2) = linspace(limit_min+0.015,limit_max,length(speeds));

f = reshape(f.',[],1);

max_F = 2;
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

csvname=strcat('data/',gen.filename,'/brtest2.csv');
csvwrite(csvname,f_vec);

end