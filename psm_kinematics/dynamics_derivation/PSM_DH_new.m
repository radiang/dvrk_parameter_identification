clear all 
close all

syms T Jw lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9 l2 l3 l4 l5 l6 l7 l8 l9 m1 m2 m3 m4 m5 m6 m7 m8 m9 q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6  qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 thet1 psi  tau1 tau2 tau3 tau4 tau5 tau6 real; 

q   = [q1 q2 q3 q4 q5 q6];
qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];

Q = [q1 q2 q3 q4 q5 q6]';
Qd = [qd1 qd2 qd3 qd4 qd5 qd6]';
Qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6]';
Tau = [tau1 tau2 tau3 tau4 tau5 tau6]';

%% Options 
dof = 6; 
%% DH Paremeters and Transforms of PSM 
p = sym(pi()); 
%My own DH from RViz and DH Paper
beta=atan((.072613-.0296)/(.2961-.1524));

a      =    [ 0     ,        0,     .150,        .516,    .2881, -.0430,        0,      0,     .0091,    .0102,    0];
alpha  =    [-p/2   ,     -p/2,        0,           0,        0,    p/2,        0,     p/2,     -p/2,        0, -p/2];
d      =    [ 0.1524,    .0296,        0,           0,        0,      0, .4162+q3,       0,        0,        0,    0];
tet    =    [ p/2   ,  -p/2+q1, -beta+q2, beta+p/2-q2,  -p/2+q2,   -p/2,      p/2,  p/2+q4,   p/2+q5,       q6, -p/2];

%1: Yaw Coordinate Frame
%2-4: Pitch Coordinate Frames
%  6: Insertion Coordinate Frame
%  7: Tool Roll Coordinate Frame
%  8: Tool Pitch Coordinate Frame
%  9: Tool Yaw Coordinate Frame
% 11: Tool end effector

for i=1:length(a)
Ti_i(1:4,1:4,i) = DH(tet(i),a(i),d(i),alpha(i));
%Ti_i(1:4,1:4,i) = dh_modified(tet(i),a(i),d(i),alpha(i));
end


k=[0;0;1];
for i=1:length(a)
    if i==1
        T(1:4,1:4,i) = Ti_i(:,:,i);
    
    else
    T(1:4,1:4,i)=T(1:4,1:4,i-1)*Ti_i(1:4,1:4,i);
    %T(1:4,1:4,j)=vpa(T(1:4,1:4,j),4);
    end
end
 
%T=double(T);

%% Transforms of counterweight
a_c = [0];
alpha_c = [0];
d_c = [0];
tet_c = [0];

for i = 1:length(a_c)
Ti_counter(:,:,i) = DH(tet_c(i),a_c(i),d_c(i),alpha_c(i)); 
if i==1 
    T_c(:,:,i)=Ti_counter(:,:,i);
else 
    T_c(:,:,i)=T_c(:,:,i-1)*Ti_counter(:,:,i);
end 
end 


%% Center of Gravity of Each Link location with DH 

%lc(i) is along common normal of frame i+1 (x_a) or along z of frame i(x_d). 
a_cg      =    [lc1, lc2, lc3, lc4, 0,   0,   0, lc8, lc9] ;
d_cg      =    [0,     0,   0,   0, 0, lc6, lc7,   0,   0];

for i=1:length(a_cg)
    alpha_cg(i)=alpha(i+1);
    tet_cg(i)=tet(i+1);
end 

for i = 1:length(a_cg)
Ti_cg(:,:,i) = DH(tet_cg(i),a_cg(i),d_cg(i),alpha_cg(i)); 
T_cg(:,:,i)  = T(:,:,i)*Ti_cg(:,:,i);
end 

%T_cg 5 does not have any mass and doesnt contribute to anything

%% Plot Joint Angles Test
q_n = [0, 0, 0, 0, 0, 0];  %Put your numeric values here
lc_n = [-.03, .150/2, .516/2, .2881/2, .4162/2, .001, .001, .001]; %Put numeric values of Cg locations

figure()
for i = 1:length(T)
        T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6],q_n);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);     
        hold on
end

for i = 1:length(T_cg)
        T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
        
        scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
        marker_id = sprintf('cg_%d',i);
        text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
        hold on
end


title('Plot transform Frames');
xlabel('x');
ylabel('y');
zlabel('z');
T_num = double(T_num);


%% Calculate Dynamic Jacobians 

%Linear Jacobian
for i=1:length(T_cg)
    Jv(1:3,1:6,i)=[diff(T_cg(1:3,4,i),q1),diff(T_cg(1:3,4,i),q2),diff(T_cg(1:3,4,i),q3),diff(T_cg(1:3,4,i),q4),diff(T_cg(1:3,4,i),q5),diff(T_cg(1:3,4,i),q6)];
end 

%Angular Jacobian
k=[0;0;1];
Jw_m(1:3,:)=[T(1:3,1:3,1)*k, T(1:3,1:3,4)*k,0,T(1:3,1:3,7)*k,T(1:3,1:3,8)*k,T(1:3,1:3,9)*k];

Jw(1:3,1:6,1)=zeros(3,6);
Jw(1:3,1:1,1)=Jw_m(1:3,1);

%Close Kinematic Chain Links
Jw(1:3,1:6,2)=zeros(3,6);
Jw(1:3,1:2,2)=[T(1:3,1:3,1)*k, T(1:3,1:3,2)*k];

Jw(1:3,1:6,3)=zeros(3,6);
Jw(1:3,1:2,3)=[T(1:3,1:3,1)*k, -T(1:3,1:3,3)*k];

Jw(1:3,1:6,4)=zeros(3,6);
Jw(1:3,1:2,4)=[T(1:3,1:3,1)*k, T(1:3,1:3,4)*k];

Jw(1:3,1:6,5)=zeros(3,6);
Jw(1:3,1:2,5)=[T(1:3,1:3,1)*k, T(1:3,1:3,5)*k];

for i=3:length(Jw_m)
    Jw(1:3,1:6,i+3)=zeros(3,6);
    Jw(1:3,1:i,i+3)=Jw_m(1:3,1:i);

end

%Jw_num(:,:,:)=double((subs(Jw(:,:,:),[q1 q2 q3 q4 q5 q6],q_n)));

%% Calculate D Matrix 

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

for i=1:length(Jv)
   %B(1:9,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*I(1:3,1:3,i)*Jw(:,:,i);
   if i==5
   B(1:6,1:6,i) = zeros(6);
   else
       B(1:6,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*T(1:3,1:3,i)*I(1:3,1:3,i)*T(1:3,1:3,i)'*Jw(:,:,i); %Not sure about transformations
   end 
end

D = zeros(dof);
for i=1:(dof+2)
    D = D + B(1:dof,1:dof,i);
end 

D = D + B(1:dof,1:dof,9);

%D=combine(D,'sincos');
D = simplify(D);
D = combine(D);










