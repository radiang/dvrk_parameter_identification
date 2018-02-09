%%Parameter Idenitification
clear all

gear12 =  56.8;
gear3  = 336.6; 

convM2cm=1;

pos=csvread('data_position.csv');
vel=csvread('data_velocity.csv');
acc=csvread('data_acceleration_filtered.csv');
tor=csvread('data_torque.csv');

hz=250;
time=linspace(0,length(vel),length(vel)+1)*1/hz;


for i=1:length(tor)-1
    torque(3*(i-1)+1)=tor(i,1)*gear12*convM2cm;
    torque(3*(i-1)+2)=tor(i,2)*gear12*convM2cm;
    torque(3*(i-1)+3)=tor(i,3)*gear3*convM2cm;
end

Q(1:3,:)=pos(:,1:3)';
Qd(1:3,:)=vel(:,1:3)';
Qdd(1:3,:)=acc(:,1:3)';

for i=1:length(Q)-1
   
q1=Q(1,i);
q2=Q(2,i);
q3=Q(3,i);

qd1=Qd(1,i);
qd2=Qd(2,i);
qd3=Qd(3,i);

qdd1=Qdd(1,i);
qdd2=Qdd(2,i);
qdd3=Qdd(3,i);   

W(1+3*(i-1),:)=[ qdd1, qdd1, qdd2,    0, qdd1, qdd2,    0, qdd1, qdd1, qdd1, qdd2,    0, qdd1, qdd2,    0, qdd1, -sin(q1), qdd1/2 + (qdd1*cos(2*q2 - (13*pi)/72))/2 - 2*qd1*qd2*sin(2*q2 - (13*pi)/72), cos(q1 + q2)/2 - cos(q1 - q2)/2, qdd1/2 + (qdd1*cos(2*q2 - (13*pi)/72))/2 - 2*qd1*qd2*sin(2*q2 - (13*pi)/72), cos(q1 + q2)/2 - cos(q1 - q2)/2, qdd1/2 + (qdd1*cos(2*q2))/2 - 2*qd1*qd2*sin(2*q2),         (381*qdd1*sin(2*q2 + (59*pi)/144))/2500 + (381*qdd1*sin((59*pi)/144))/2500 + (381*qd1*qd2*cos(2*q2 + (59*pi)/144))/625, cos(q1 + q2)/4 - cos(q1 - q2)/4 + (3^(1/2)*sin(q1 + q2))/4 + (3^(1/2)*sin(q1 - q2))/4, 0, 0, 0, (145161*qdd1)/12500000 - (381*cos(q1 - q2))/5000 + (381*cos(q1 + q2))/5000 + (145161*qdd1*cos(2*q2 - (13*pi)/72))/12500000 - (145161*qd1*qd2*sin(2*q2 - (13*pi)/72))/3125000, (145161*qdd1)/8000000 - (4953*cos(q1 - q2))/40000 + (4953*cos(q1 + q2))/40000 + (145161*qdd1*cos(2*q2))/16000000 + (381*3^(1/2)*sin(q1 + q2))/8000 + (381*3^(1/2)*sin(q1 - q2))/8000 + (145161*3^(1/2)*qdd1*sin(2*q2))/16000000 - (145161*qd1*qd2*sin(2*q2))/4000000 + (145161*3^(1/2)*qd1*qd2*cos(2*q2))/4000000, (145161*qdd1)/12500000 + (145161*qdd1*cos(2*q2 - (13*pi)/72))/12500000 - (145161*qd1*qd2*sin(2*q2 - (13*pi)/72))/3125000, (142893*qdd1)/6250000 - (381*cos(q1 - q2))/5000 + (3*cos(2*q1))/40 + (381*cos(q1 + q2))/5000 + (3*3^(1/2)*sin(2*q1))/40 - (q3*cos(2*q1))/2 + (9*qdd1*cos(2*q2))/800 + (145161*qdd1*cos(2*q2 - (13*pi)/72))/12500000 + (1143*qdd1*sin(2*q2 + (59*pi)/144))/50000 - (3*q3*qdd1)/20 - (3*qd1*qd3)/10 + (q3^2*qdd1)/2 + (1143*qdd1*sin((59*pi)/144))/50000 - (381*q3*qdd1*sin((59*pi)/144))/2500 - (381*qd1*qd3*sin((59*pi)/144))/1250 - (3^(1/2)*q3*sin(2*q1))/2 - (3*q3*qdd1*cos(2*q2))/20 - (3*qd1*qd3*cos(2*q2))/10 + (1143*qd1*qd2*cos(2*q2 + (59*pi)/144))/12500 - (9*qd1*qd2*sin(2*q2))/200 - (381*q3*qdd1*sin(2*q2 + (59*pi)/144))/2500 - (145161*qd1*qd2*sin(2*q2 - (13*pi)/72))/3125000 - (381*qd1*qd3*sin(2*q2 + (59*pi)/144))/1250 + 2*q3*qd1*qd3 + (q3^2*qdd1*cos(2*q2))/2 - 2*q3^2*qd1*qd2*sin(2*q2) + 2*q3*qd1*qd3*cos(2*q2) - (381*q3*qd1*qd2*cos(2*q2 + (59*pi)/144))/625 + (3*q3*qd1*qd2*sin(2*q2))/5];
W(2+3*(i-1),:)=[    0,    0, qdd1, qdd2,    0, qdd1, qdd2,    0,    0,    0, qdd1, qdd2,    0, qdd1, qdd2,    0,        0,                                         sin(2*q2 - (13*pi)/72)*qd1^2 + qdd2, cos(q1 - q2)/2 + cos(q1 + q2)/2,                                         sin(2*q2 - (13*pi)/72)*qd1^2 + qdd2, cos(q1 - q2)/2 + cos(q1 + q2)/2,                            sin(2*q2)*qd1^2 + qdd2, - (381*qdd2*sin(2*q2 + (59*pi)/144))/1250 - (381*qd1^2*cos(2*q2 + (59*pi)/144))/1250 - (381*qd2^2*cos(2*q2 + (59*pi)/144))/625, cos(q1 - q2)/4 + cos(q1 + q2)/4 + (3^(1/2)*sin(q1 + q2))/4 - (3^(1/2)*sin(q1 - q2))/4, 0, 0, 0,                                                    (145161*sin(2*q2 - (13*pi)/72)*qd1^2)/6250000 + (145161*qdd2)/6250000 + (381*cos(q1 - q2))/5000 + (381*cos(q1 + q2))/5000,                                                                                   (145161*qdd2)/4000000 + (4953*cos(q1 - q2))/40000 + (4953*cos(q1 + q2))/40000 + (381*3^(1/2)*sin(q1 + q2))/8000 + (145161*qd1^2*sin(2*q2))/8000000 - (381*3^(1/2)*sin(q1 - q2))/8000 - (145161*3^(1/2)*qd1^2*cos(2*q2))/8000000,                               (145161*sin(2*q2 - (13*pi)/72)*qd1^2)/6250000 + (145161*qdd2)/6250000 + (381*cos(q2))/2500,                                                                                                                                                                                                                                                                                                                             (142893*qdd2)/3125000 + (381*cos(q1 - q2))/5000 + (381*cos(q1 + q2))/5000 - (381*qdd3*cos(2*q2 + (59*pi)/144))/2500 + (3*qdd3*sin(2*q2))/20 - (3*q3*qdd2)/10 - (3*qd2*qd3)/5 - (1143*qd1^2*cos(2*q2 + (59*pi)/144))/25000 + (9*qd1^2*sin(2*q2))/400 - 2*qd3^2*sin(2*q2) + (145161*qd1^2*sin(2*q2 - (13*pi)/72))/6250000 + q3^2*qdd2 + (1143*qdd2*sin((59*pi)/144))/25000 + q3^2*qd1^2*sin(2*q2) - (381*q3*qdd2*sin((59*pi)/144))/1250 - (381*qd2*qd3*sin((59*pi)/144))/625 - q3*qdd3*sin(2*q2) + 4*q3*qd2*qd3 + (381*q3*qd1^2*cos(2*q2 + (59*pi)/144))/1250 - (3*q3*qd1^2*sin(2*q2))/10];
W(3+3*(i-1),:)=[    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,        0,                                                                           0,                               0,                                                                           0,                               0,                                                 0,                                                                                                                              0,                                                                                     0, 0, 0, 0,                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                 0,                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                         qdd3 + cos(2*q1 + pi/6)/2 - (381*qdd2*cos(2*q2 + (59*pi)/144))/2500 + (3*qdd2*sin(2*q2))/20 + (381*qd1^2*sin((59*pi)/144))/2500 + (381*qd2^2*sin((59*pi)/144))/1250 + 3^(1/2)/4 + (3*qd1^2*cos(2*q2))/20 + (3*qd2^2*cos(2*q2))/5 + (381*qd1^2*sin(2*q2 + (59*pi)/144))/2500 + (381*qd2^2*sin(2*q2 + (59*pi)/144))/625 - q3*qd1^2 - 2*q3*qd2^2 + (3*qd1^2)/20 + (3*qd2^2)/10 - q3*qdd2*sin(2*q2) - q3*qd1^2*cos(2*q2) - 4*q3*qd2^2*cos(2*q2)];
 

end

%P_W=inv(W'*W)*W';

[U,De,V]=svd(W);
De_e=De(1:31,1:31);
De(1:31,1:31)=inv(De_e);
P_W=V*De'*U';

Par=P_W*torque'; 
csvwrite('parameters.csv',Par)