function [Cond]=new_cond(z,Y,n,dof_max,q,qd,qdd)

Ys2=Y;

ts=0.1;
tf=2;

for j = 1:dof_max
for t=1:n
a = sprintf('z(%d)',(j-1)*n+t);
b = sprintf('array(%d,%d)',[j, t+1]);
eval(strcat(b,'=',a));

a2 = sprintf('z(%d)',dof_max*n+(j-1)*n+t);
b2 = sprintf('array_v(%d,%d)',[j,t+1]);
eval(strcat(b2,'=',a2));

end 
end

A0=zeros(dof_max,21);

for dof=1:dof_max
[Q_data(dof,:),Qd_data(dof,:),Qdd_data(dof,:),T]=Trajectory_f(array(dof,:),array_v(dof,:),tf,ts);
end

W = zeros(length(Q_data(1,:))*dof_max,21);

for i=1:length(Q_data(1,:))
   
q1=Q_data(1,i);
q2=Q_data(2,i);
q3=Q_data(3,i);

qd1=Qd_data(1,i);
qd2=Qd_data(2,i);
qd3=Qd_data(3,i);
  

qdd1=Qdd_data(1,i);
qdd2=Qdd_data(2,i);
qdd3=Qdd_data(3,i);
%     
% 
%   t2 = q3*q3;
%   t3 = q2*2.0;
%   t4 = t3-5.816747949334241E-1;
%   t5 = cos(t4);
%   t6 = q1+q2-2.908373974667121E-1;
%   t7 = sin(t6);
%   t8 = q1-q2+2.908373974667121E-1;
%   t9 = sin(t8);
%   t10 = sin(t4);
%   t11 = q2-2.908373974667121E-1;
%   t12 = q1-q2;
%   t13 = q1+q2;
%   t14 = cos(t3);
%   t15 = sin(t3);
%   t16 = t3-2.908373974667121E-1;
%   t17 = cos(t16);
%   t18 = sin(t16);
%   t19 = cos(t12);
%   t20 = cos(t13);
%   t21 = sin(2.908373974667121E-1);
%   t22 = cos(t11);
%   t23 = qd2*qd2;
%   t24 = sin(t11);
%   t25 = q2-5.816747949334241E-1;
%   t26 = qd1*qd1;
%   t27 = cos(t6);
%   t28 = t27*(4.9E1/5.0);
%   t29 = cos(t8);
%   t30 = t19*2.107E-1;
%   t31 = sin(t12);
%   t32 = sin(t13);
%   t33 = cos(2.908373974667121E-1);
%   t34 = t19*(4.9E1/1.0E1);
%   t35 = sin(q2);
%   t36 = sin(t25);
%   
%   A0(1,1) = q3*t7*(-4.9E1/1.0E1)-q3*t9*(4.9E1/1.0E1)+qdd1*t2*(1.0/2.0)+q3*qd1*qd3*2.0+qdd1*t2*t5*(1.0/2.0)+q3*qd1*qd3*t5*2.0-qd1*qd2*t2*t10*2.0;
%   A0(1,2) = t7*(-4.9E1/1.0E1)-t9*(4.9E1/1.0E1)+q3*qdd1+qd1*qd3*2.0+q3*qdd1*t5+qd1*qd3*t5*2.0-q3*qd1*qd2*t10*4.0;
%   A0(1,3) = t28-t29*(4.9E1/5.0)+q3*qdd1*t10*2.0+qd1*qd3*t10*4.0+q3*qd1*qd2*t5*8.0;
%   A0(1,5) = -q1;
%   A0(1,8) = -qd1;
%   A0(1,11) = -(qd1/abs(qd1));
%   A0(1,12) = qdd2*t24+t22*t23*2.0;
%   A0(1,13) = qdd1*t15+qd1*qd2*t14*4.0;
%   A0(1,14) = qdd1*5.3675305E-2-t7*(1.47E2/2.0E2)-t9*(1.47E2/2.0E2)-t20*2.107E-1+t30-t31*1.41169-t32*1.41169+qdd1*t5*(9.0/8.0E2)+qdd1*t14*4.0576305E-2-qdd1*t15*1.23883E-2+qdd1*t17*4.3215E-2-qdd1*t18*6.45E-3-qdd1*t21*6.45E-3+qdd1*t33*4.3215E-2-qd1*qd2*t10*(9.0/2.0E2)-qd1*qd2*t14*4.95532E-2-qd1*qd2*t15*1.6230522E-1-qd1*qd2*t17*2.58E-2-qd1*qd2*t18*1.7286E-1;
%   A0(1,15) = -qdd2;
%   A0(1,17) = qdd1*(1.0/2.0)-qdd1*t5*(1.0/2.0)+qd1*qd2*t10*2.0;
%   A0(1,18) = t20*(-4.9E1/1.0E1)+t34-qdd1*t18*(3.0/2.0E1)-qdd1*t21*(3.0/2.0E1)-qd1*qd2*t17*(3.0/5.0);
%   A0(1,19) = -qdd2*t22+t23*t24*2.0;
%   A0(1,20) = qdd1*t10+qd1*qd2*t5*4.0;
%   A0(1,21) = sin(q1-2.908373974667121E-1)*(-4.9E1/1.0E1)-sin(q1+2.908373974667121E-1)*(4.9E1/1.0E1)+qdd1*cos(q2)*(3.0/2.0E1)+qdd1*cos(t25)*(3.0/2.0E1)-qd1*qd2*t35*(3.0/1.0E1)-qd1*qd2*t36*(3.0/1.0E1);
%   A0(2,1) = q3*t7*(-4.9E1/1.0E1)+q3*t9*(4.9E1/1.0E1)+qdd2*t2+q3*qd2*qd3*4.0+t2*t10*t26;
%   A0(2,2) = t7*(-4.9E1/1.0E1)+t9*(4.9E1/1.0E1)+q3*qdd2*2.0+qd2*qd3*4.0+q3*t10*t26*2.0;
%   A0(2,3) = qdd3*2.0+t28+t29*(4.9E1/5.0)-q3*t5*t26*4.0;
%   A0(2,4) = -q2;
%   A0(2,7) = -qd2;
%   A0(2,10) = -(qd2/abs(qd2));
%   A0(2,12) = qdd1*t24;
%   A0(2,13) = t14*t26*-2.0;
%   A0(2,14) = qdd2*1.0735061E-1-t7*(1.47E2/2.0E2)+t9*(1.47E2/2.0E2)-t20*2.107E-1-t30+t31*1.41169-t32*1.41169-qdd2*t21*1.29E-2+qdd2*t33*8.643E-2+t10*t26*(9.0/4.0E2)+t14*t26*2.47766E-2+t15*t26*8.115261E-2+t17*t26*1.29E-2+t18*t26*8.643E-2;
%   A0(2,15) = -qdd1;
%   A0(2,16) = qdd2;
%   A0(2,17) = -t10*t26;
%   A0(2,18) = t20*(-4.9E1/1.0E1)-t34-qdd2*t21*(3.0/1.0E1)+t17*t26*(3.0/1.0E1);
%   A0(2,19) = -qdd1*t22;
%   A0(2,20) = t5*t26*-2.0;
%   A0(2,21) = t26*t35*(3.0/2.0E1)+t26*t36*(3.0/2.0E1);
%   A0(3,1) = qdd3-q3*t23*2.0-q3*t26+t22*cos(q1)*(4.9E1/5.0)-q3*t5*t26;
%   A0(3,2) = t23*-2.0-t26-t5*t26;
%   A0(3,3) = qdd2*2.0-t10*t26*2.0;
%   A0(3,6) = -qd3;
%   A0(3,8) = -(qd3/abs(qd3));
%     
 W(1+(i-1)*dof_max,:) = [ (q3^2*qdd1)/2 - (49*q3*sin(q1 - q2 + 5239260779425957/18014398509481984))/10 - (49*q3*sin(q1 + q2 - 5239260779425957/18014398509481984))/10 + (q3^2*qdd1*cos(2*q2 - 5239260779425957/9007199254740992))/2 + 2*q3*qd1*qd3 - 2*q3^2*qd1*qd2*sin(2*q2 - 5239260779425957/9007199254740992) + 2*q3*qd1*qd3*cos(2*q2 - 5239260779425957/9007199254740992), q3*qdd1 - (49*sin(q1 - q2 + 5239260779425957/18014398509481984))/10 - (49*sin(q1 + q2 - 5239260779425957/18014398509481984))/10 + 2*qd1*qd3 + q3*qdd1*cos(2*q2 - 5239260779425957/9007199254740992) + 2*qd1*qd3*cos(2*q2 - 5239260779425957/9007199254740992) - 4*q3*qd1*qd2*sin(2*q2 - 5239260779425957/9007199254740992), (49*cos(q1 + q2 - 5239260779425957/18014398509481984))/5 - (49*cos(q1 - q2 + 5239260779425957/18014398509481984))/5 + 2*q3*qdd1*sin(2*q2 - 5239260779425957/9007199254740992) + 4*qd1*qd3*sin(2*q2 - 5239260779425957/9007199254740992) + 8*q3*qd1*qd2*cos(2*q2 - 5239260779425957/9007199254740992),   0, -q1,    0,    0, -qd1,          0,          0, -sign(qd1), 2*cos(q2 - 5239260779425957/18014398509481984)*qd2^2 + qdd2*sin(q2 - 5239260779425957/18014398509481984), qdd1*sin(2*q2) + 4*qd1*qd2*cos(2*q2), (10735061*qdd1)/200000000 + (2107*cos(q1 - q2))/10000 - (141169*sin(q1 - q2))/100000 - (147*sin(q1 + q2 - 5239260779425957/18014398509481984))/200 - (147*sin(q1 - q2 + 5239260779425957/18014398509481984))/200 - (2107*cos(q1 + q2))/10000 - (141169*sin(q1 + q2))/100000 + (8115261*qdd1*cos(2*q2))/200000000 - (123883*qdd1*sin(2*q2))/10000000 + (8643*qdd1*cos(5239260779425957/18014398509481984))/200000 - (129*qdd1*sin(5239260779425957/18014398509481984))/20000 + (9*qdd1*cos(2*q2 - 5239260779425957/9007199254740992))/800 + (8643*qdd1*cos(2*q2 - 5239260779425957/18014398509481984))/200000 - (129*qdd1*sin(2*q2 - 5239260779425957/18014398509481984))/20000 - (129*qd1*qd2*cos(2*q2 - 5239260779425957/18014398509481984))/5000 - (9*qd1*qd2*sin(2*q2 - 5239260779425957/9007199254740992))/200 - (8643*qd1*qd2*sin(2*q2 - 5239260779425957/18014398509481984))/50000 - (123883*qd1*qd2*cos(2*q2))/2500000 - (8115261*qd1*qd2*sin(2*q2))/50000000, -qdd2,    0, qdd1/2 - (qdd1*cos(2*q2 - 5239260779425957/9007199254740992))/2 + 2*qd1*qd2*sin(2*q2 - 5239260779425957/9007199254740992), (49*cos(q1 - q2))/10 - (49*cos(q1 + q2))/10 - (3*qdd1*sin(5239260779425957/18014398509481984))/20 - (3*qdd1*sin(2*q2 - 5239260779425957/18014398509481984))/20 - (3*qd1*qd2*cos(2*q2 - 5239260779425957/18014398509481984))/5, 2*sin(q2 - 5239260779425957/18014398509481984)*qd2^2 - qdd2*cos(q2 - 5239260779425957/18014398509481984), qdd1*sin(2*q2 - 5239260779425957/9007199254740992) + 4*qd1*qd2*cos(2*q2 - 5239260779425957/9007199254740992), (3*qdd1*cos(q2 - 5239260779425957/9007199254740992))/20 - (49*sin(q1 + 5239260779425957/18014398509481984))/10 - (49*sin(q1 - 5239260779425957/18014398509481984))/10 + (3*qdd1*cos(q2))/20 - (3*qd1*qd2*sin(q2))/10 - (3*qd1*qd2*sin(q2 - 5239260779425957/9007199254740992))/10];
 W(2+(i-1)*dof_max,:) =                                        [                                                                                                                                    (49*q3*sin(q1 - q2 + 5239260779425957/18014398509481984))/10 + q3^2*qdd2 - (49*q3*sin(q1 + q2 - 5239260779425957/18014398509481984))/10 + q3^2*qd1^2*sin(2*q2 - 5239260779425957/9007199254740992) + 4*q3*qd2*qd3,                                                                                                                   2*q3*sin(2*q2 - 5239260779425957/9007199254740992)*qd1^2 - (49*sin(q1 + q2 - 5239260779425957/18014398509481984))/10 + (49*sin(q1 - q2 + 5239260779425957/18014398509481984))/10 + 2*q3*qdd2 + 4*qd2*qd3,                                                                                                            - 4*q3*cos(2*q2 - 5239260779425957/9007199254740992)*qd1^2 + 2*qdd3 + (49*cos(q1 + q2 - 5239260779425957/18014398509481984))/5 + (49*cos(q1 - q2 + 5239260779425957/18014398509481984))/5, -q2,   0,    0, -qd2,    0,          0, -sign(qd2),          0,                                                        qdd1*sin(q2 - 5239260779425957/18014398509481984),                   -2*qd1^2*cos(2*q2),                                                                                                                                                                                                                                                                                   (10735061*qdd2)/100000000 - (2107*cos(q1 - q2))/10000 + (141169*sin(q1 - q2))/100000 - (147*sin(q1 + q2 - 5239260779425957/18014398509481984))/200 + (147*sin(q1 - q2 + 5239260779425957/18014398509481984))/200 - (2107*cos(q1 + q2))/10000 - (141169*sin(q1 + q2))/100000 + (129*qd1^2*cos(2*q2 - 5239260779425957/18014398509481984))/10000 + (9*qd1^2*sin(2*q2 - 5239260779425957/9007199254740992))/400 + (8643*qd1^2*sin(2*q2 - 5239260779425957/18014398509481984))/100000 + (123883*qd1^2*cos(2*q2))/5000000 + (8115261*qd1^2*sin(2*q2))/100000000 + (8643*qdd2*cos(5239260779425957/18014398509481984))/100000 - (129*qdd2*sin(5239260779425957/18014398509481984))/10000, -qdd1, qdd2,                                                                      -qd1^2*sin(2*q2 - 5239260779425957/9007199254740992),                                                               (3*cos(2*q2 - 5239260779425957/18014398509481984)*qd1^2)/10 - (49*cos(q1 - q2))/10 - (49*cos(q1 + q2))/10 - (3*qdd2*sin(5239260779425957/18014398509481984))/10,                                                       -qdd1*cos(q2 - 5239260779425957/18014398509481984),                                                       -2*qd1^2*cos(2*q2 - 5239260779425957/9007199254740992),                                                                                                                                                                                                   (3*qd1^2*sin(q2))/20 + (3*qd1^2*sin(q2 - 5239260779425957/9007199254740992))/20];
 W(3+(i-1)*dof_max,:) =                                        [                                                                                                                                                                                                  qdd3 - q3*qd1^2 - 2*q3*qd2^2 + (49*cos(q2 - 5239260779425957/18014398509481984)*cos(q1))/5 - q3*qd1^2*cos(2*q2 - 5239260779425957/9007199254740992),                                                                                                                                                                                                                                                    - qd1^2*cos(2*q2 - 5239260779425957/9007199254740992) - qd1^2 - 2*qd2^2,                                                                                                                                                                                                                                     - 2*sin(2*q2 - 5239260779425957/9007199254740992)*qd1^2 + 2*qdd2,   0,   0, -qd3,    0,    0, -sign(qd3),          0,          0,                                                                                                        0,                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,     0,    0,                                                                                                                         0,                                                                                                                                                                                                                             0,                                                                                                        0,                                                                                                            0,                                                                                                                                                                                                                                                                                 0];
    
% A0=[];
% W(1+(i-1)*dof_max:dof_max+(i-1)*dof_max,:)=subs(Ys2, [q(1:dof_max) qd(1:dof_max) qdd(1:dof_max)],[transpose(Q(1:dof_max,i)), transpose(Qd(1:dof_max,i)), transpose(Qdd(1:dof_max,i))]);
end


Cond = double(cond(W));


end