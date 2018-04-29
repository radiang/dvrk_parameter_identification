function [x,v,ac,T]=Trajectory_quintic(theta,dtheta,ddtheta,tf,ts,plot_set)% theta = [qa, qb, ... qn] , dtheta = [ qa', qb', ..., qc'] , minimal 2 values. 

if nargin > 4
    plot_on = plot_set; 
else 
    plot_on = 0;
end 
    
%plot_on = 1;

Q=[theta;
   dtheta;ddtheta];

t0 = 0;
t=t0:ts:tf;

r=size(Q,2)-1;

T_mat = [1, t0, t0^2,     t0^3,      t0^4,      t0^5;
         0,  1,  2*t0, 3*(t0^2),  4*(t0^3),  5*(t0^4);
         0,  0,     2,      6*t0, 12*(t0^2), 20*(t0^3);
         1, tf, tf^2,     tf^3,      tf^4,      tf^5;
         0,  1,  2*tf, 3*(tf^2),  4*(tf^3),  5*(tf^4);
         0,  0,     2,      6*tf, 12*(tf^2), 20*(tf^3)];
     
for i=1:r
qa=Q(1,i);
qb=Q(1,i+1);
dqa=Q(2,i);
dqb=Q(2,i+1);
ddqa=Q(3,i);
ddqb=Q(3,i+1);

que = [qa,dqa,ddqa, qb, dqb, ddqb].';

a(:,i)=inv(T_mat)*que;

%Fifth Polynomial trajectory based on paper


end

for i=1:1:r
for k=1:length(t) 
%this is the constraints to fmincon
x(k+length(t)*(i-1))=a(1,i)+a(2,i)*t(k)+ a(3,i)*t(k).^2+a(4,i)*t(k).^3+a(5,i)*t(k).^4+a(6,i)*t(k).^5;
v(k+length(t)*(i-1))=a(2,i)+ 2*a(3,i)*t(k) +3*a(4,i)*t(k).^2+4*a(5,i)*t(k).^3+5*a(6,i)*t(k).^4;
ac(k+length(t)*(i-1))=2*a(3,i) +6*a(4,i)*t(k)+12*a(5,i)*t(k).^2+20*a(6,i)*t(k).^3;
end
end

%Deleting extra value at interval
 for i=r-1:-1:1
 x(length(t)*(i))=[];
 v(length(t)*(i))=[];
 ac(length(t)*(i))=[];
 end

%Plotting

T=0:ts:tf*r;
if plot_on == 1

 figure();
 
 subplot(3,1,1);
 plot(T,x,'LineWidth',3);
 title('Position (rad)')
 grid
 
 %figure('Name','Velocity (degree/s)');
 subplot(3,1,2);
 plot(T,v,'LineWidth',3);
 title('Velocity (rad/s)')
  grid
 
 %figure('Name','Acceleration (degree/s^2)');
 subplot(3,1,3);
 plot(T,ac,'LineWidth',3);
 title('Acceleration (rad/s^2)')
 grid
end 

end