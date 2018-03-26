
%Dynamics Derivation based on DH parameter of Luigi Villani Paper IROS
%2017. 
%Implemented by Radian Azhar, October 2017

%% Dynamics Derivation 6 DOF
clear all
syms T Jw lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9 l2 l3 l4 l5 l6 l7 l8 l9 m1 m2 m3 m4 m5 m6 m7 m8 m9 q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6  qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 thet1 psi  tau1 tau2 tau3 tau4 tau5 tau6 real; 
q   = [q1 q2 q3 q4 q5 q6];
qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];

%Choose dof number
dof = 6;
%Known Kinematic Parameters
l2=0.150; l3=0.515; l4=0.150; l5=0.0100; l6=0.005; l7=0.010; thet1=3.14/6;

%Center of Mass Locations
%Link 1: X(:,1)
%Link 2: X(:,2)-(:,4)
%Link 3: X(:,5)

X(1:3,1)=[lc1*cos(thet1)*sin(q1); lc1*sin(thet1); lc1*cos(thet1)*cos(q1)];
X(1:3,2)=[lc2*cos(q2)*sin(q1); lc2*sin(q2); lc2*cos(q2)*cos(q1)];
X(1:3,3)=[l2*cos(q2)*sin(q1); l2*sin(q2)+lc2; l2*cos(q2)*cos(q1)];
X(1:3,4)=[lc4*cos(q2)*sin(q1);lc4*sin(q2)+lc3; lc4*cos(q2)*cos(q1)];
X(1:3,5)=[(lc5-q3)*cos(q2)*sin(q1);(lc5-q3)*sin(q2)+l3; (lc5-q3)*cos(q2)*cos(q1)];
X(1:3,6)=[(lc6-q3)*cos(q2)*sin(q1);(lc6-q3)*sin(q2)+l3; (lc6-q3)*cos(q2)*cos(q1)];
X(1:3,7)=[(-l5-q3)*cos(q2)*sin(q1)-lc7*sin(q4)*sin(q5);(-l5-q3)*sin(q2)+l3-lc7*cos(q4)*sin(q5);(-l5-q3)*cos(q2)*cos(q1)-lc7*cos(q5)];
X(1:3,8)=[(-l5-q3)*cos(q2)*sin(q1)-l7*sin(q4)*sin(q5)+lc8*cos(q4)*sin(q6);(-l5-q3)*sin(q2)+l3-l7*cos(q4)*sin(q5)-lc8*sin(q4);(-l5-q3)*cos(q2)*cos(q1)-l7*cos(q5)-lc8*cos(q6)];
X(1:3,9)=[(lc9+q3)*cos(q2)*sin(q1);(lc9+q3)*sin(q2);(lc9+q3)*cos(q2)*cos(q1)];

%DH Parameters
a=[0 0 0.2 0.5 0 0 0 0.009 0 0];
alpha=[sym(pi()/2) sym(-pi()/2) 0 0 sym(-pi()/2) 0 sym(pi()/2) sym(-pi()/2) sym(-pi()/2) 0];
d=[0 0 0 0 0 q3 0 0 0 lc9+q3];
tet=[0 q1 q2 -q2 q2 0 q4 q5 0 0];
%Rotation Matrix from DH 
for i=1:length(a)
        Ti_i(1:4,1:4,i)=[cos(tet(i)), -sin(tet(i))*cos(alpha(i)), sin(tet(i))*sin(alpha(i)) , a(i)*cos(tet(i));
                        sin(tet(i)) , cos(tet(i))*cos(alpha(i)) , -cos(tet(i))*sin(alpha(i)), a(i)*sin(tet(i));
                        0           , sin(alpha(i))             , cos(alpha(i))             , d(i);
                        0           , 0                         ,  0                        ,   1];
end

k=[0;0;1];
for i=1:length(Ti_i)
    T(1:4,1:4,i)=eye(4);
    for j=1:i
    T(1:4,1:4,i)=vpa(T(1:4,1:4,i)*Ti_i(1:4,1:4,j),2);
    end 
    %T(1:4,1:4,j)=vpa(T(1:4,1:4,j),4);
end


%% D Matrix 

%Angular Jacobian
Jw_m(1:3,:)=[T(1:3,1:3,1)*k, T(1:3,1:3,4)*k,0,T(1:3,1:3,5)*k,T(1:3,1:3,6)*k,T(1:3,1:3,7)*k];

Jw(1:3,1:6,1)=zeros(3,6);
Jw(1:3,1:1,1)=Jw_m(1:3,1);

%Close Kinematic Chain Links
Jw(1:3,1:6,2)=zeros(3,6);
Jw(1:3,1:2,2)=[T(1:3,1:3,1)*k, T(1:3,1:3,2)*k];

Jw(1:3,1:6,3)=zeros(3,6);
Jw(1:3,1:2,3)=[T(1:3,1:3,1)*k, T(1:3,1:3,3)*k];

Jw(1:3,1:6,4)=zeros(3,6);
Jw(1:3,1:2,4)=[T(1:3,1:3,1)*k, T(1:3,1:3,4)*k];

for i=3:length(Jw_m)
    Jw(1:3,1:6,i+2)=zeros(3,6);
    Jw(1:3,1:i,i+2)=Jw_m(1:3,1:i);

end
%Counterweight
Jw(1:3,1:6,9)=zeros(3,6);
Jw(1:3,1:2,9)=[T(1:3,1:3,1)*k, T(1:3,1:3,2)*k];



%Linear Jacobian Notation
for i=1:length(X)
    
    Jv(1:3,1:6,i)=[diff(X(:,i),q1),diff(X(:,i),q2),diff(X(:,i),q3),diff(X(:,i),q4),diff(X(:,i),q5),diff(X(:,i),q6)];

end 

%Inertia Matrix notation
Ixx = sym('I%d_xx', [9 1]);
Iyy = sym('I%d_yy', [9 1]);
Izz = sym('I%d_zz', [9 1]);
Ixy = sym('I%d_xy', [9 1]);
Ixz = sym('I%d_xz', [9 1]);
Iyz = sym('I%d_yz', [9 1]);

for i=1:1:length(Ixx)
    
    I(1:3,1:3,i)=[Ixx(i),Ixy(i),Ixz(i); 
              Ixy(i),Iyy(i),Iyz(i);
              Ixz(i),Iyz(i),Izz(i);]; 
end

%Inertial Frame Rotation
M=[m1 m2 m3 m4 m5 m6 m7 m8 m9];

for i=1:1:length(Jv)
   %B(1:6,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*I(1:3,1:3,i)*Jw(:,:,i);
   
   B(1:6,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*T(1:3,1:3,i)*I(1:3,1:3,i)*T(1:3,1:3,i)'*Jw(:,:,i);
end

D = eye(dof);
for i=1:(dof+2)
    D = D + B(1:dof,1:dof,i);
end 

D = D + B(1:dof,1:dof,9);

%D=combine(D,'sincos');
D = simplify(D);
D = combine(D);

Q = [q1 q2 q3 q4 q5 q6]';
Qd = [qd1 qd2 qd3 qd4 qd5 qd6]';
Qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6]';
Tau = [tau1 tau2 tau3 tau4 tau5 tau6]';


%% Crystoffel Symbols
for i=1:dof
    for j=1:dof
        for k=1:dof
            c(i,j,k)=diff(D(k,j),Q(i))+diff(D(k,i),Q(j))-diff(D(i,j),Q(k));
        end
    end
end
C=sym(zeros(dof,dof));
    for j=1:dof
        for k=1:dof
            for i=1:dof
                C(k,j)= C(k,j) + c(i,j,k)*Qd(i);
                %C(k,j)=(c(1,j,k)*Qd(1)+c(2,j,k)*Qd(2)+c(3,j,k)*Qd(3)+c(4,j,k)*Qd(4)+c(5,j,k)*Qd(5)+c(6,j,k)*Qd(6));
            end
        end
    end

    
%% Potential Energy
P(1)= m1*lc1*cos(thet1)*cos(q1);
P(2)= m2*lc2*cos(q2)*cos(q1);
P(3)= m3*l2*cos(q2)*cos(q1);
P(4)= m4*lc4*cos(q2)*cos(q1);
P(5)= m5*(lc5-q3)*cos(q2)*cos(q1);
P(6)= m6*(lc6-q3)*cos(q2)*cos(q1);
P(7)= m7*((-l5-q3)*cos(q2)*cos(q1)-lc7*cos(q5));
P(8)= m8*((-l5-q3)*cos(q2)*cos(q1)-lc7*cos(q5)-lc8*cos(q6));
P(9)= m9*((lc9+q3)*cos(q2)*cos(q1));

P_tog=sym(zeros(1));

for i=1:dof
   P_tog = P_tog + P(i);
end

   %P_tog=m1*lc1*cos(thet1)*cos(q1)+m2*lc2*cos(q2)*cos(q1)+m3*l2*cos(q2)*cos(q1)+m4*lc4*cos(q2)*cos(q1)+m5*(lc5-q3)*cos(q2)*cos(q1)+m6*(lc6-q3)*cos(q2)*cos(q1)+m7*((-l5-q3)*cos(q2)*cos(q1)-lc7*cos(q5))+m8*((-l5-q3)*cos(q2)*cos(q1)-lc7*cos(q5)-lc8*cos(q6))+m9*((lc9+q3)*cos(q2)*cos(q1));

Psi=sym(zeros(dof,1));

for i=1:dof 
Psi(i,1)=diff(P_tog,Q(i));
end


%% Put together
Dt=D*Qdd(1:dof)+C*Qd(1:dof)+Psi;
Dt=simplify(Dt);
%T=combine(T,'sincos');
Dt=collect(Dt,[Ixx,Ixy,Ixz,Iyy,Iyz,Izz, lc1, lc2, lc3, lc4, lc5, lc6, lc7, lc8, lc9,m1, m2, m3, m4, m5, m6, m7, m8, m9]);

%Dt=collect(Dt,[Ixx(1),Ixx(2),Ixz(2),Izz(2),Ixx(3),Ixz(3),Izz(3),Ixx(4),Ixx(5),Ixx(6),Ixz(6),Izz(6),lc1, lc2, lc3,lc4, lc5, lc6,lc7, lc8, lc9 m1, m2, m3, m4, m5, m6, m7, m8, m9]);

syms lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc4_2_m4 lc4_m4  lc5_2_m5 lc5_m5 lc6_2_m6 lc6_m6 lc7_2_m7 lc7_m7  lc8_2_m8 lc8_m8 lc9_2_m9 lc9_m9 lc7_m8
Dt = subs(Dt, [ lc1^2*m1 lc1*m1 lc2^2*m2 lc2*m2 lc4^2*m4 lc4*m4 lc5^2*m5 lc5*m5 lc6^2*m6 lc6*m6 lc7^2*m7 lc7*m7 lc8^2*m8 lc8*m8 lc9^2*m9 lc9*m9 lc7*m8], [lc1_2_m1 lc1_m1 lc2_2_m2 lc2_m2 lc4_2_m4 lc4_m4 lc5_2_m5 lc5_m5 lc6_2_m6 lc6_m6 lc7_2_m7 lc7_m7  lc8_2_m8 lc8_m8 lc9_2_m9 lc9_m9 lc7_m8]);


%% Regressor Matrix Form
Par=symvar(Dt);

%Delete q's from Par
check=q;
check(end+1:end+length(qd)) = qd;
check(end+1:end+length(qdd)) = qdd;
[bullshit,ind]=ismember(check,Par);

 for k=length(ind):-1:1
       if ind(k)~= 0
       Par(ind(k))=[];
       end
 end
 
 
%Par(end-17:end)=[]; 
[Y, tau]=equationsToMatrix(Dt == Tau(1:dof), Par);

%% Lumping Parameters

%finding the linear combinations
%[Ys1, Ys2, Par1, Par2]=lumping_parameters(Par);

% %% Trajectory Optimization 
% 
% [x, v, cond_save]=traj_opt_rand(Ys2,size(Ys2,2));


%Show the Results
disp('Resulting Regressor Matrix: ')
%Ys2

disp('Resulting Identifiable Parameters: ')
%Par2

