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

M=[m1 m2 m3 m4 m5 m6 m7 m8 m9];
Lc=[lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9];

%% Options 
%Degrees of Freedom of robot
dof = 6; 

%calculate dynamics?? 
dynamic = 1;

%Joint Angles
q_n = [0, 0, 0, 0, 0, 0];  %Put your numeric values here

%Mass Locations
lc_n = [-.03, .150/2, .516/2, .2881/2, .4162/2, .001, .001, .001]; %Put numeric values of Cg locations

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

% %lc(i) is along common normal of frame i+1 (x_a) or along z of frame i(x_d). 
% a_cg      =    [lc1, lc2, lc3, lc4, 0,   0,   0, lc8, lc9] ;
% d_cg      =    [0,     0,   0,   0, 0, lc6, lc7,   0,   0];
% 
% for i=1:length(a_cg)
%     alpha_cg(i)=alpha(i+1);
%     tet_cg(i)=tet(i+1);
% end 
% 
% for i = 1:length(a_cg)
% Ti_cg(:,:,i) = DH(tet_cg(i),a_cg(i),d_cg(i),alpha_cg(i)); 
% T_cg(:,:,i)  = T(:,:,i)*Ti_cg(:,:,i);
% end 
% 
% %T_cg 5 does not have any mass and doesnt contribute to anything
% 

%% Let's try a new Center of Mass 
l_cg = sym('l_cg%d_%d', [9 3]);

map = [q1,q2,-q2,q2,0,q3,q4,q5,q6];
for i = 1:length(l_cg) 
p_cg(i,:)  = T(:,:,i)*DH(map(i),0,0,0)*transpose([l_cg(i,:), 1]);
end 


%% Plot Joint Angles Test

figure()
for i = 1:length(T)
        T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6],q_n);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);     
        hold on
end


% 
% for i = 1:length(T_cg)
%         T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
%         
%         scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
%         marker_id = sprintf('cg_%d',i);
%         text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
%         hold on
% end

 for i = 1:length(p_cg)
         p_cg_num(i,:)=subs(p_cg(i,:),[q l_cg(i,:)], [q_n, 0, 0, .1]);
%         
         scatter3(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),'*');
         marker_id = sprintf('cg_%d',i);
         text(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),marker_id);
         hold on
 end
title('Plot transform Frames');
xlabel('x');
ylabel('y');
zlabel('z');
T_num = double(T_num);



%% Calculate Dynamic Jacobians 

%Linear Jacobian
% for i=1:length(T_cg)
%     Jv_cg(1:3,1:6,i)=[diff(T_cg(1:3,4,i),q1),diff(T_cg(1:3,4,i),q2),diff(T_cg(1:3,4,i),q3),diff(T_cg(1:3,4,i),q4),diff(T_cg(1:3,4,i),q5),diff(T_cg(1:3,4,i),q6)];
% end 

 for i=1:length(p_cg)
     Jv_cg(1:3,1:6,i)=[diff(transpose(p_cg(i,1:3)),q1),diff(transpose(p_cg(i,1:3)),q2),diff(transpose(p_cg(i,1:3)),q3),diff(transpose(p_cg(i,1:3)),q4),diff(transpose(p_cg(i,1:3)),q5),diff(transpose(p_cg(i,1:3)),q6)];
 end 

%Angular Jacobian
k=[0;0;1];
Jw_m(1:3,:)=[T(1:3,1:3,1)*k, T(1:3,1:3,4)*k,0 ,T(1:3,1:3,7)*k,T(1:3,1:3,8)*k,T(1:3,1:3,9)*k];

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

for i=3:length(T)
    Jw(1:3,1:6,i+3)=zeros(3,6);
    if(i<7)
    Jw(1:3,1:i,i+3)=Jw_m(1:3,1:i);
    else
    Jw(1:3,1:6,i+3)=Jw_m(1:3,1:6);
    end
end


%Jw_num(:,:,:)=double((subs(Jw(:,:,:),[q1 q2 q3 q4 q5 q6],q_n)));

%% Calculate End Link Jacobians
%Linear Jacobian

for i=1:length(T)
    Jv(1:3,1:6,i)=[diff(T(1:3,4,i),q1),diff(T(1:3,4,i),q2),diff(T(1:3,4,i),q3),diff(T(1:3,4,i),q4),diff(T(1:3,4,i),q5),diff(T(1:3,4,i),q6)];
end 

J_end = [Jv(:,:,11);Jw(:,:,11)]; %End Effector Jacobian
%% Calculate D Matrix 


if dynamic==1

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


for i=1:length(Jv_cg)
   %B(1:9,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*I(1:3,1:3,i)*Jw(:,:,i);
   if i==5
   B(1:6,1:6,i) = zeros(6);
   else
       B(1:6,1:6,i)=M(i)*Jv_cg(:,:,i)'*Jv_cg(:,:,i)+Jw(:,:,i)'*T(1:3,1:3,i)*I(1:3,1:3,i)*T(1:3,1:3,i)'*Jw(:,:,i); %Not sure about transformations
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
    
%%  Potential Energy
  P=sym(zeros(1,length(p_cg)));
  P_tog=sym(zeros(1));
  Psi = sym(zeros(dof,1));
  
  for i=1:length(p_cg)
  P(i) = M(i)*p_cg(i,3);
  
  if (i==5)
      P(i)=0;
  end
  
  P_tog = P_tog + P(i);
  
  end 
  
%     P=sym(zeros(1,length(T_cg)));
%   P_tog=sym(zeros(1));
%   Psi = sym(zeros(dof,1));
%   
%   for i=1:length(T_cg)
%   P(i) = M(i)*T_cg(3,4,i);
%   
%   if (i==5)
%       P(i)=0;
%   end
%   
%   P_tog = P_tog + P(i);
%   
%   end 
  
  
for i=1:dof 
Psi(i,1)=diff(P_tog,Q(i));
end

%% Put together
Dt=D*Qdd(1:dof)+C*Qd(1:dof)+Psi;
Dt=simplify(Dt);

save('temp.mat')
%T=combine(T,'sincos');

%Dt=collect(Dt,[Ixx,Ixy,Ixz,Iyy,Iyz,Izz, lc1, lc2, lc3, lc4, lc5, lc6, lc7, lc8, lc9,m1, m2, m3, m4, m5, m6, m7, m8, m9]);
  
 
% 
% lcX_2_mX = sym('lc%d_2_m%d', [9 9]);
% lcX_2_mX = reshape(lcX_2_mX,1,[]);
% 
% lcX_mX = sym('lc%d_m%d', [9 9]);
% lcX_mX = reshape(lcX_mX,1,[]);
% 
% 
% for i = 1:length(M)
%     if i==1
%     LcX_mX_mult = Lc*M(i);
%     LcX2_mX_mult = (Lc.^2)*M(i);
%     else
%     LcX_mX_mult = [LcX_mX_mult, Lc*M(i)];
%     LcX2_mX_mult = [LcX2_mX_mult, (Lc.^2)*M(i)];
%     end 
%  end 
% 
% Dt = subs(Dt, LcX2_mX_mult,lcX_2_mX);
% 
% Dt = subs(Dt, LcX_mX_mult ,lcX_mX);
% 
% 
% 
% %% Regressor Matrix Form
% Par=symvar(Dt);
% 
% %Delete q's from Par
% check=q;
% check(end+1:end+length(qd)) = qd;
% check(end+1:end+length(qdd)) = qdd;
% [bullshit,ind]=ismember(check,Par);
% 
%  for k=length(ind):-1:1
%        if ind(k)~= 0
%        Par(ind(k))=[];
%        end
%  end
%  
%  
% %Par(end-17:end)=[]; 
% [Y, tau]=equationsToMatrix(Dt == Tau(1:dof), Par);

%% Lumping Parameters

%finding the linear combinations
%[Ys1, Ys2, Par1, Par2]=lumping_parameters_new(Y,Par);

% %% Trajectory Optimization 
% 
% [x, v, cond_save]=traj_opt_rand(Ys2,size(Ys2,2));


%Show the Results
disp('Resulting Regressor Matrix: ')
%Ys2

disp('Resulting Identifiable Parameters: ')
%Par2

%save('psm_new_dynamics.mat')
end