clear all
syms lc1 lc2 l2m lc3 lc4 lc5 lc6 l2 l3 l4 l5 l6 l7 m1 m2 m3 m4 m5 m6 m7 q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3 thet psi  tau1 tau2 tau3 real; 

l2=6*0.0254; l3=6*0.0254; l4=0.100; l5=0.100; %l6=50; 
l2m=7.5*0.0254;  l7=0.150; thet=60*pi()/180; psi=73.75*pi()/180; 
% q1=1;
% q2=1;
% q3=1;


%D matrix
%Jacobian Notation
Jv(1:3,1:3,1)=[0,                                 0,             0;                               -lc1*sin(q1),                                       0,               0;                              lc1*cos(q1),                                         0,                0];
Jv(1:3,1:3,2)=[0,                  -lc2*sin(psi+q2),             0;                   -lc2*sin(psi+q2)*sin(q1),                 lc2*cos(psi+q2)*cos(q1),               0;                  lc2*sin(psi+q2)*cos(q1),                   lc2*cos(psi+q2)*sin(q1),                0];
Jv(1:3,1:3,3)=[0,                  -lc3*sin(psi+q2),             0;                   -lc3*sin(psi+q2)*sin(q1),                 lc3*cos(psi+q2)*cos(q1),               0;                  lc3*sin(psi+q2)*cos(q1),                   lc3*cos(psi+q2)*sin(q1),                0];
Jv(1:3,1:3,4)=[0,                   -l2*sin(psi+q2),             0;                    -l2*sin(psi+q2)*sin(q1),                  l2*cos(psi+q2)*cos(q1),               0;                   l2*sin(psi+q2)*cos(q1),                    l2*cos(psi+q2)*sin(q1),                0]; 
Jv(1:3,1:3,5)=[0,                 -l2m*sin(thet+q2),             0;                  -sin(q1)*l2m*sin(thet+q2),                cos(q1)*l2m*cos(thet+q2),               0;                 cos(q1)*l2m*sin(thet+q2),                  sin(q1)*l2m*cos(thet+q2),                0];
Jv(1:3,1:3,6)=[0,       -l2*sin(psi+q2)+lc6*cos(q2),             0;      -sin(q1)*(l2*sin(psi+q2)+lc6*cos(q2)),    cos(q1)*(l2*cos(psi+q2)-lc6*sin(q2)),               0;     cos(q1)*(l2*sin(psi+q2)+lc6*cos(q2)),      sin(q1)*(l2*cos(psi+q2)-lc6*sin(q2)),                0];
Jv(1:3,1:3,7)=[0, -(l2*sin(psi+q2)+(l7-q3)*cos(q2)),      -sin(q2);  -(l2*sin(psi+q2)+(l7-q3)*cos(q2))*sin(q1),(l2*cos(psi+q2)-(l7-q3)*sin(q2))*cos(q1),-cos(q2)*cos(q1); (l2*sin(psi+q2)+(l7-q3)*cos(q2))*cos(q1),(l2*cos(psi+q2)-(l7-q3)*sin(q2))*sin(q1)  , -cos(q2)*sin(q1)];

Jw(1:3,1:3,1)=[1 0 0; 0 0 0; 0 0 0];
Jw(1:3,1:3,2)=[1 0 0; 0 0 0; 0 1 0];
Jw(1:3,1:3,3)=Jw(1:3,1:3,2);
Jw(1:3,1:3,4)=[1 0 0; 0 0 0; 0 0 0];
Jw(1:3,1:3,5)=Jw(1:3,1:3,4);
Jw(1:3,1:3,6)=Jw(1:3,1:3,2);
Jw(1:3,1:3,7)=Jw(1:3,1:3,2);


%Inertia Matrix notation
Ixx = sym('I%d_xx', [7 1]);
Iyy = sym('I%d_yy', [7 1]);
Izz = sym('I%d_zz', [7 1]);
Ixy = sym('I%d_xy', [7 1]);
Ixz = sym('I%d_xz', [7 1]);
Iyz = sym('I%d_yz', [7 1]);

for i=1:1:7
    
    I(1:3,1:3,i)=[Ixx(i),Ixy(i),Ixz(i); 
              Ixy(i),Iyy(i),Iyz(i);
              Ixz(i),Iyz(i),Izz(i);];
          
end
%Inertial Frame Rotation


M=[m1 m2 m3 m4 m5 m6 m7];

for i=1:1:7
    
   B(1:3,1:3,i)=M(i)*Jv(1:3,1:3,i)'*Jv(1:3,1:3,i)+Jw(1:3,1:3,i)'*I(1:3,1:3,i)*Jw(1:3,1:3,i);
   
end

D=B(1:3,1:3,1)+B(1:3,1:3,2)+B(1:3,1:3,3)+B(1:3,1:3,4)+B(1:3,1:3,5)+B(1:3,1:3,6)+B(1:3,1:3,7);
%D=combine(D,'sincos');
D=simplify(D);
D=combine(D);

Q=[q1 q2 q3]';
Qd=[qd1 qd2 qd3]';
Qdd=[qdd1 qdd2 qdd3]';
Tau=[tau1 tau2 tau3]';


% Q=[1 2 3]';
% Qd=[1 2 3]';
% Qdd=[1 2 3]';
% Tau=[1 2 3]';
%%
%Crystoffel Symbols
for i=1:1:3
    for j=1:1:3
        for k=1:1:3
            c(i,j,k)=diff(D(k,j),Q(i))+diff(D(k,i),Q(j))-diff(D(i,j),Q(k));
        end
    end
end

    for j=1:1:3
        for k=1:1:3
            C(k,j)=(c(1,j,k)*Qd(1)+c(2,j,k)*Qd(2)+c(3,j,k)*Qd(3));
        end
    end


%%
%Potential Energy
P=m1*lc1*cos(q1)+m2*lc2*sin(psi+q2)*cos(q1)+m3*lc3*sin(psi+q2)*cos(q1)+m4*l2*sin(psi+q2)*cos(q1)+m5*l2m*sin(q2+thet)*cos(q1)+m6*(l2*sin(psi+q2)+lc6*cos(q2))*cos(q1)+m7*(l2*sin(psi+q2)+(l7-q3)*cos(q2))*cos(q1);
for i=1:1:3
Psi(i,1)=diff(P,Q(i));
end



%% Put together
T=D*Qdd+C*Qd+Psi;
T=simplify(T);
%T=combine(T,'sincos');
T=collect(T,[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),Ixx(7),Ixz(7),Izz(7),lc1, lc2, lc3, lc6, m1, m2, m3, m4, m5, m6, m7]);

syms lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc3_2_m3 lc3_m3 lc6_2_m6 lc6_m6
T = subs(T, [ lc1^2*m1 lc1*m1 lc2^2*m2 lc2*m2 lc3^2*m3 lc3*m3 lc6^2*m6 lc6*m6], [lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc3_2_m3 lc3_m3 lc6_2_m6 lc6_m6]);


%% Regessor Matrix Form

%Par=[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),Ixx(7),Ixz(7),Izz(7), lc1^2*m1, lc1*m1, lc2^2*m2, lc2*m2, lc3^2*m3, lc3*m3, lc6^2*m6,lc6*m6, lc6, m4, m5, m6, m7];
Par=[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),Ixx(7),Ixz(7),Izz(7), lc1_2_m1, lc1_m1, lc2_2_m2 ,lc2_m2 ,lc3_2_m3, lc3_m3 ,lc6_2_m6, lc6_m6, lc6, m1, m2, m3, m4, m5, m6, m7];
[Y, tau]=equationsToMatrix(T == Tau, Par);

%%PseudoInverse and Least Square Solution
P_Y=inv(Y'*Y)*Y';
