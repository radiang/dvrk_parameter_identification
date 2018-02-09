clear all
syms lc1 lc2 l2m lc3 lc4 lc5 lc6 l2 l3 l4 l5 l6 l7 m1 m2 m3 m4 m5 m6 m7 q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3 thet psi  tau1 tau2 tau3 real; 

l2=6*0.0254; l3=6*0.0254; l4=0.100; l5=0.100; %l6=50; 
l2m=7.5*0.0254;  thet=60*pi()/180; psi=73.75*pi()/180; 
% q1=1;
% q2=1;
% q3=1;


%D matrix
%Jacobian Notation
Jv(1:3,1:2,1)=[0,                                 0;                               -lc1*sin(q1),                                       0;                              lc1*cos(q1),                                         0];
Jv(1:3,1:2,2)=[0,                  -lc2*sin(psi+q2);                   -lc2*sin(psi+q2)*sin(q1),                 lc2*cos(psi+q2)*cos(q1);                  lc2*sin(psi+q2)*cos(q1),                   lc2*cos(psi+q2)*sin(q1)];
Jv(1:3,1:2,3)=[0,                  -lc3*sin(psi+q2);                   -lc3*sin(psi+q2)*sin(q1),                 lc3*cos(psi+q2)*cos(q1);                  lc3*sin(psi+q2)*cos(q1),                   lc3*cos(psi+q2)*sin(q1)];
Jv(1:3,1:2,4)=[0,                   -l2*sin(psi+q2);                    -l2*sin(psi+q2)*sin(q1),                  l2*cos(psi+q2)*cos(q1);                   l2*sin(psi+q2)*cos(q1),                    l2*cos(psi+q2)*sin(q1)]; 
Jv(1:3,1:2,5)=[0,                 -l2m*sin(thet+q2);                  -sin(q1)*l2m*sin(thet+q2),                cos(q1)*l2m*cos(thet+q2);                 cos(q1)*l2m*sin(thet+q2),                  sin(q1)*l2m*cos(thet+q2)];
Jv(1:3,1:2,6)=[0,       -l2*sin(psi+q2)+lc6*cos(q2);      -sin(q1)*(l2*sin(psi+q2)+lc6*cos(q2)),    cos(q1)*(l2*cos(psi+q2)-lc6*sin(q2));     cos(q1)*(l2*sin(psi+q2)+lc6*cos(q2)),      sin(q1)*(l2*cos(psi+q2)-lc6*sin(q2))];

Jw(1:3,1:2,1)=[1 0; 0 0; 0 0];
Jw(1:3,1:2,2)=[1 0; 0 0; 0 1];
Jw(1:3,1:2,3)=Jw(1:3,1:2,2);
Jw(1:3,1:2,4)=[1 0; 0 0; 0 0];
Jw(1:3,1:2,5)=Jw(1:3,1:2,4);
Jw(1:3,1:2,6)=Jw(1:3,1:2,2);


%Inertia Matrix notation
Ixx = sym('I%d_xx', [6 1]);
Iyy = sym('I%d_yy', [6 1]);
Izz = sym('I%d_zz', [6 1]);
Ixy = sym('I%d_xy', [6 1]);
Ixz = sym('I%d_xz', [6 1]);
Iyz = sym('I%d_yz', [6 1]);

for i=1:1:6
    
    I(1:3,1:3,i)=[Ixx(i),Ixy(i),Ixz(i); 
              Ixy(i),Iyy(i),Iyz(i);
              Ixz(i),Iyz(i),Izz(i);];
          
end
%Inertial Frame Rotation


M=[m1 m2 m3 m4 m5 m6];

for i=1:1:6
    
   B(1:2,1:2,i)=M(i)*Jv(1:3,1:2,i)'*Jv(1:3,1:2,i)+Jw(1:3,1:2,i)'*I(1:3,1:3,i)*Jw(1:3,1:2,i);
   
end

D=B(1:2,1:2,1)+B(1:2,1:2,2)+B(1:2,1:2,3)+B(1:2,1:2,4)+B(1:2,1:2,5)+B(1:2,1:2,6);
%D=combine(D,'sincos');
D=simplify(D);
D=combine(D);

Q=[q1 q2]';
Qd=[qd1 qd2]';
Qdd=[qdd1 qdd2]';
Tau=[tau1 tau2]';


%%
%Crystoffel Symbols
for i=1:1:2
    for j=1:1:2
        for k=1:1:2
            c(i,j,k)=diff(D(k,j),Q(i))+diff(D(k,i),Q(j))-diff(D(i,j),Q(k));
        end
    end
end

    for j=1:1:2
        for k=1:1:2
            C(k,j)=(c(1,j,k)*Qd(1)+c(2,j,k)*Qd(2));
        end
    end


%%
%Potential Energy
P=m1*lc1*cos(q1)+m2*lc2*sin(psi+q2)*cos(q1)+m3*lc3*sin(psi+q2)*cos(q1)+m4*l2*sin(psi+q2)*cos(q1)+m5*l2m*sin(q2+thet)*cos(q1)+m6*(l2*sin(psi+q2)+lc6*cos(q2))*cos(q1);
for i=1:1:2
Psi(i,1)=diff(P,Q(i));
end



%% Put together
T=D*Qdd+C*Qd+Psi;
T=simplify(T);
%T=combine(T,'sincos');
T=collect(T,[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),lc1, lc2, lc3, lc6, m1, m2, m3, m4, m5, m6]);

syms lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc3_2_m3 lc3_m3 lc6_2_m6 lc6_m6
T = subs(T, [ lc1^2*m1 lc1*m1 lc2^2*m2 lc2*m2 lc3^2*m3 lc3*m3 lc6^2*m6 lc6*m6], [lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc3_2_m3 lc3_m3 lc6_2_m6 lc6_m6]);


%% Regessor Matrix Form

%Par=[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),Ixx(7),Ixz(7),Izz(7), lc1^2*m1, lc1*m1, lc2^2*m2, lc2*m2, lc3^2*m3, lc3*m3, lc6^2*m6,lc6*m6, lc6, m4, m5, m6, m7];
Par=[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6), lc1_2_m1, lc1_m1, lc2_2_m2 ,lc2_m2 ,lc3_2_m3, lc3_m3 ,lc6_2_m6, lc6_m6, m4, m5, m6];
[Y, tau]=equationsToMatrix(T == Tau, Par);

%BASE PARAMETER GROUPING BASE ON DELETING COLUMNS
Yb=Y;
%delete [qdd1;0], Essentially lumping the parameters to same column
%[qdd1;0]*(par_a+par_b)
% Yb(:,2)=[];
% Yb(:,5)=[];
% Yb(:,8:10)=[];
% Yb(:,13)=[];
% 
% %delete [qdd2;qdd1], Essentially lumping the parameters to same column
% %[qdd2;qdd1]*(par_a+par_b)
% Yb(:,6)=[];
% Yb(:,11)=[];
% 
% %delete [0;qdd2], Essentially lumping the parameters to same column
% %[0;qdd2]*(par_a+par_b)
% Yb(:,7)=[];
% Yb(:,12)=[];

%Delete from end to not screw up indexes of the other
Yb(:,5:13)=[];
Yb(:,2)=[];

%Lumped Parameters Become
Par_b=Par;
Par_b(:,5:13)=[];
Par_b(:,2)=[];
Par_b(:,1)=Par(:,1)+Par(:,2)+Par(:,5)+Par(:,8)+Par(:,9)+Par(:,10)+Par(:,13);
Par_b(:,2)=Par(:,2)+Par(:,6)+Par(:,11);
Par_b(:,3)=Par(:,3)+Par(:,7)+Par(:,12);

%GROUPING BASED ON COEFFICIENT MULTIPLICATION OF PARAMETERS
%Sum of the Columns
 Yc=Yb;
 Yc(:,1)=Yb(:,1)+Yb(:,4); Yc(:,4)=[];
 
 %Lumped Parameters Become
 Par_c=Par_b;
 
 syms alpha1 
 Par_c(:,1)=Par_b(:,1)+alpha1*Par_b(:,4); Par_c(:,4)=[];

%MORE GROUPING BASED ON COEFFICIENT MULTIPLICATION OF PARAMETERS, Checked
%from the null(W) of trajectory optimization, with random data, there are
%certain dependent columns
Yd=Yc;
Yd(:,4)=Yc(:,4)+Yc(:,6);
Yd(:,5)=Yc(:,5)+Yc(:,7);
Yd(:,10)=Yc(:,10)+Yc(:,12);
Yd(:,12)=[];
Yd(:,7)=[];
Yd(:,6)=[];

%Lumped Parameters Become
Par_d=Par_c;

syms alpha2 alpha3 alpha4
Par_d(:,4)=Par_c(:,4)+alpha2*Par_c(:,6);
Par_d(:,5)=Par_c(:,5)+alpha3*Par_c(:,7);
Par_d(:,10)=Par_c(:,10)+alpha4*Par_c(:,12);
Par_d(:,12)=[];Par_d(:,7)=[]; Par_d(:,6)=[];

%MORE GROUPING BASED ON COEFFICIENT MULTIPLICATION OF PARAMETERS, Checked
%from the null(W) of trajectory optimization, with random data, there are
%certain dependent columns
Ye=Yd;
Ye(:,5)=Yd(:,5)+Yd(:,8);
Ye(:,8)=[];

%Lumped Parameters Become
Par_e=Par_d;

syms alpha5
Par_e(:,5)=Par_d(:,5)+alpha5*Par_d(:,8);
Par_e(:,8)=[];




%%PseudoInverse and Least Square Solution
P_Y=inv(Y'*Y)*Y';
