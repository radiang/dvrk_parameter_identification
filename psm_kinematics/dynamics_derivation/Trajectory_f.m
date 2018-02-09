function [x,v,ac,T]=Trajectory_f(theta,dtheta,tf,ts)
Q=[theta;
   dtheta];

t=0:ts:tf;
r=length(Q)-1;

for i=1:r
qa=Q(1,i);
qb=Q(1,i+1);
dqa=Q(2,i);
dqb=Q(2,i+1);

A= qb-qa;
a(1,i)=qa;
a(2,i)=dqa;
a(3,i)=0;
a(4,i)=10*A*tf^-3-(6*dqa+4*dqb)*tf^-2;
a(5,i)=-15*A*tf^-4+(8*dqa+7*dqb)*tf^-3; 
a(6,i)=6*A*tf^-5-3*(dqa+dqb)*tf^-4;

end

for i=1:1:r
for k=1:length(t) 
%this is the constraints to fmincon
x(k+length(t)*(i-1))=a(1,i)+a(2,i)*t(k)+ a(3,i)*t(k).^2+a(4,i)*t(k).^3+a(5,i)*t(k).^4+a(6,i)*t(k).^5;
v(k+length(t)*(i-1))=a(2,i)+ 2*a(3,i)*t(k) +3*a(4,i)*t(k).^2+4*a(5,i)*t(k).^3+5*a(6,i)*t(k).^4;
ac(k+length(t)*(i-1))=2*a(3,i) +6*a(4,i)*t(k)+12*a(5,i)*t(k).^2+20*a(6,i)*t(k).^3;
end
end

for i=1:1:r-1
x(length(t)*(i))=[];
v(length(t)*(i))=[];
ac(length(t)*(i))=[];
end

T=0:ts:tf*r;
% figure('Name','Position (degree)');
% plot(T,x,'LineWidth',3);
% title('Position (degree)')
% grid
% 
% figure('Name','Velocity (degree/s)');
% plot(T,v,'LineWidth',3);
% title('Velocity (degree/s)')
%  grid
% 
% figure('Name','Acceleration (degree/s^2)');
% plot(T,ac,'LineWidth',3);
% title('Acceleration (degree/s^2)')
% grid

end