%Dynamics Derivation based on DH parameter of Luigi Villani Paper IROS
%2017. 
%Implemented by Radian Azhar, October 2017

%% Dynamics Derivation 6 DOF
clear all
close all

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

%Numeric Values 
q1n = 0; q2n = 0; q3n = 0; q4n = 0; q5n = 0; q6n =0; lc1n = 0.1; lc2n = l2/2; lc3n = l3/2; lc4n = l4/2; lc5n = l5/2; lc6n = l6/2; lc7n = l7/2; lc8n = 0.04; lc9n = 0.2;

figure()
 for i = 1:length(X)
         X_num(:,i)=subs(X(:,i),[q1, q2, q3, q4, q5, q6, lc1, lc2, lc3, lc4, lc5, lc6, lc7, lc8, lc9], [q1n, q2n, q3n, q4n, q5n, q6n, lc1n, lc2n, lc3n, lc4n, lc5n, lc6n, lc7n, lc8n, lc9n]);     
        scatter3(X_num(1,i),X_num(2,i),X_num(3,i));
        
        marker_id = sprintf('%d',i);
        text(X_num(1,i),X_num(2,i),X_num(3,i),marker_id);
        
        
        hold on
 end 
 
 title('Plot cg Frames');
xlabel('x');
ylabel('y');
zlabel('z');

 
p = sym(pi()); 
%DH Parameters old

% a      =    [0  ,    0, 0.2, 0.5,    0,  0,   0, 0.009,    0,      0];
% alpha  =    [p/2, -p/2,   0,   0, -p/2,  0, p/2,  -p/2, -p/2,      0];
% d      =    [0  ,    0,   0,   0,    0, q3,   0,     0,    0, lc9+q3];
% tet    =    [0  ,   q1,  q2, -q2,   q2,  0,  q4,    q5,    0,      0];

%DH Parameters from paper
% a      =    [  0,   0.2, 0.5,    0,  0,   0, 0.009,    0,      0,  0];
% alpha  =    [-p/2,   0,   0,  -p/2,  0, p/2,  -p/2, -p/2,   -p/2,  0];
% d      =    [    0,   0,   0,    0, q3,   0,     0,    0,      0, lc9+q3];
% tet    =    [   q1,  q2,  -q2,   q2,  0,  q4,    q5,   q6,      0,  0];

%My own DH from RViz and DH Paper
a      =    [  0.0296 ];
alpha  =    [  -p/2];
d      =    [   0.1524 ];
tet    =    [   p/2];

%Rotation Matrix from DH  (correct)

for i=1:length(a)
Ti_i(1:4,1:4,i) = DH(tet(i),a(i),d(i),alpha(i));
%Ti_i(1:4,1:4,i) = dh_modified(tet(i),a(i),d(i),alpha(i));
end


k=[0;0;1];
T(1:4,1:4,1)=eye(4);
for i=1:length(Ti_i)
    T(1:4,1:4,i)=vpa(T(1:4,1:4,i)*Ti_i(1:4,1:4,i),2)
    %T(1:4,1:4,j)=vpa(T(1:4,1:4,j),4);
end

T(1:4,1:4,9)=T(:,:,2)*Ti_i(:,:,9);
T(1:4,1:4,10)=T(:,:,9)*Ti_i(:,:,10);

figure()
for i = 1:length(T)
        T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6 lc9], [0, p/4, 0, 0, 0, 0, 0.2]);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);
        
        
        hold on
end

title('Plot transform Frames');
xlabel('x');
ylabel('y');
zlabel('z');
%zaxis([-1 1]);












