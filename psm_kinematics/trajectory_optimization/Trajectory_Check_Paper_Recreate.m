%%Parameter Idenitification
%Gautier and Khalil 1992 Parameter Identification
clear all
tf=2.5;
ts=0.05;
g=9.8;

theta(1,:) = [0, -1.8,-2,-2,-2, -2, -2.2, -2.5, -3, -3, -3.2, -3.2, -3, -3 ,-2.8, -2.2,-2.2, -2.2, -2.2, -2, -2, -1.8, -1.8, -1.5, -1, 0];
dtheta(1,:)= [0, -1, 0.5, 0, -0.5, -0.5, -0.5, 0, 0.5, 0, -0.3, 0.5, -0.7, 0, 0.5, 0, 0, 0.2, 0, -0.5, -0.1, 0, 0.3, 0.4, 0, 0];
[Q(1,:),Qd(1,:),Qdd(1,:),T]=Trajectory_f(theta(1,:),dtheta(1,:),tf,ts);

theta(2,:) = [0,1,2,2,2,2.2,2.2,2.6,2.2,2.2,2.4,1.8,1.8,1.8,1,0.8,0.8,1,0.8,0.6,0.6,0.8,0.6,0.8,1,0];
dtheta(2,:)= [0,1,0,-0.4,0.2,0,0,2,-1,0,0.2,-0.2,0,-0.2,-1,0.2,0.4,0,0.3,-1,-1.5,-0.3,-0.2,0,-0.2,0];
[Q(2,:),Qd(2,:),Qdd(2,:),T]=Trajectory_f(theta(2,:),dtheta(2,:),tf,ts);

W=zeros(2*length(Q),3);


%Test Trajectory for Least squares solution
for i=1:length(Q)
   
q1=Q(1,i);
q2=Q(2,i);


qd1=Qd(1,i);
qd2=Qd(2,i);


qdd1=Qdd(1,i);
qdd2=Qdd(2,i);
 
%(1)
%W(1+2*(i-1),:)=[ qdd1, qdd1 + qdd2, g*cos(q1), (cos(q2)*qd2^2)/5 + (2*qd1*cos(q2)*qd2)/5 + g*sin(q1 + q2) + (qdd1*sin(q2))/5 + (qdd2*sin(q2))/10, qdd1/100 + (g*cos(q1))/10];
%W(2+2*(i-1),:)=[    0, qdd1 + qdd2,         0,                                            g*sin(q1 + q2) - (qd1^2*cos(q2))/5 + (qdd1*sin(q2))/10,                         0];
 
%From above W matrix, put columns 3 and 5 and add to column 1
%(2)
W(1+2*(i-1),:)=[qdd1+g*cos(q1)+qdd1/100+(g*cos(q1))/10,qdd1 + qdd2,(cos(q2)*qd2^2)/5+(2*qd1*cos(q2)*qd2)/5+g*sin(q1 + q2)+(qdd1*sin(q2))/5+(qdd2*sin(q2))/10];
W(2+2*(i-1),:)=[                                     0,qdd1 + qdd2,                                            g*sin(q1 + q2)-(qd1^2*cos(q2))/5+(qdd1*sin(q2))/10];
 
end

%From combining the W matrix (1) to (2), The Identifiable Parameters are: 
Par=[Izz(1)+lc1_2_m1+lc1_m1+m2,Izz(2)+lc2_2_m2,lc2_m2];

%Results see that trajectory is less important than putting in the correct
%matrix form. Problem is finding the lc2 element. 

Cond=cond(W,2);
Condf=cond(W'*W,'fro');
%P_W=inv(W'*W)*W';

% [U,De,V]=svd(W);
% De_e=De(1:31,1:31);
% De(1:31,1:31)=inv(De_e);
% P_W=V*De'*U';
% 
% Torque = linspace(1,4,93);
% Par=P_W*Torque'; 
