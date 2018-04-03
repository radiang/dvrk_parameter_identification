clear all 
close all

syms T Jw lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9 l2 l3 l4 l5 l6 l7 l8 l9 m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6  qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 thet1 psi  tau1 tau2 tau3 tau4 tau5 tau6 real; 

q   = [q1 q2 q3 q4 q5 q6];
qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];

Q = [q1 q2 q3 q4 q5 q6]';
Qd = [qd1 qd2 qd3 qd4 qd5 qd6]';
Qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6]';
Tau = [tau1 tau2 tau3 tau4 tau5 tau6]';

M=[m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11];
Lc=[lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9];


filename='data/lcg3_inplane_lumped6to2';


%% Options 
%Degrees of Freedom of robot
dof = 6; 

%calculate dynamics?? 
dynamic = 1;

%Gravity 
g =9.8; %m/s^2
%Joint Angles
q_n = [0, .5, .02, 0, 0, 0];  %Put your numeric values here

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
% 12: Counterweight Pitch
% 13: Counterweight Insertion

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
%% Syms Stuff

num = 11;

l_cg = sym('l_cg%d_%d', [num 3], 'real');
l_cg = [l_cg,ones(num,1)];
%Inertia Matrix notation
Ixx = sym('I%d_xx', [11 1],'real');
Iyy = sym('I%d_yy', [11 1],'real');
Izz = sym('I%d_zz', [11 1],'real');
Ixy = sym('I%d_xy', [11 1],'real');
Ixz = sym('I%d_xz', [11 1],'real');
Iyz = sym('I%d_yz', [11 1],'real');

for i=1:1:length(Ixx)
    
    I(1:3,1:3,i)=[Ixx(i),Ixy(i),Ixz(i); 
              Ixy(i),Iyy(i),Iyz(i);
              Ixz(i),Iyz(i),Izz(i);]; 
end

%% Make Stuff Zero

%Assume counterweight pitch goes to zero 
M(10)=0;
I(1:3,1:3,10) = zeros(3);
l_cg(10,:) = zeros(1,4);

% M(11)=0;
% I(1:3,1:3,11) = zeros(3);
% l_cg(11,:) = zeros(1,3);

%In Plane Masses
l_cg(1,1) = 0;
l_cg(2,3) = 0;
l_cg(3,3) = 0;
l_cg(4,3) = 0;
%l_cg(6,2) = 0;
%l_cg(7,1) = 0;
%l_cg(8,3) = 0;
l_cg(9,2) = 0;
%l_cg(11,2) = 0;

%Lump Gripper stuff 
M(7)=0;
I(1:3,1:3,7) = zeros(3);
l_cg(7,:) = zeros(1,4);

M(8)=0;
I(1:3,1:3,8) = zeros(3);
l_cg(8,:) = zeros(1,4);

%Lump More Stuff
M(6)=0;
I(1:3,1:3,6) = zeros(3);
l_cg(6,:) = zeros(1,4);

M(11)=0;
I(1:3,1:3,11) = zeros(3);
l_cg(11,:) = zeros(1,4);

%In Axis Masses
% l_cg(1,3) = 0;
% l_cg(1,2) = d(2);
% l_cg(2,2:3) = 0;
% l_cg(3,2:3) = 0;
% l_cg(4,2:3) = 0;
% l_cg(6,1:2) = 0;
% l_cg(7,1) = 0;
% l_cg(7,3) = 0;
% l_cg(8,2:3) = 0;
% l_cg(9,2:3) = 0;
% l_cg(11,1:2) = 0;


%% Let's try a new Center of Mass 

map = [q1,q2,-q2,q2,0,q3,q4,q5,q6];
%INPLANE
for i = 1:length(l_cg)-2 
p_cg(i,:)  = T(:,:,i)*DH(map(i),0,0,0)*transpose(l_cg(i,:));
end 

%INAXIS
% for i = 1:length(l_cg)-2 
% p_cg(i,:)  = T(:,:,i+1)*transpose([l_cg(i,:), 1]);
% end 

counter_num = 2;
Tee_cg(:,:,1)= T(:,:,2)*DH(map(2),0,0,0);
Tee_cg(:,:,2)= T(:,:,2)*DH(-p/2-beta+q2,l_cg(11,1),0,-p/2)*DH(0,0,map(6),0);

 p_cg(end+1,:) = Tee_cg(:,:,1)*transpose(l_cg(10,:)); %Pitch Counterweight 
 p_cg(end+1,:) = Tee_cg(:,:,2)*transpose(l_cg(11,:)); %Insertion Counterweight 

 
%% Plot Joint Angles Test
% 
% figure()
% for i = 1:length(T)
%         T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6],q_n);
%         
%         scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
%         marker_id = sprintf('%d',i);
%         text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);     
%         hold on
% end
% 
% 
% % 
% % for i = 1:length(T_cg)
% %         T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
% %         
% %         scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
% %         marker_id = sprintf('cg_%d',i);
% %         text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
% %         hold on
% % end
% 
%  for i = 1:length(p_cg)
%          p_cg_num(i,:)=subs(p_cg(i,:),[q l_cg(i,:)], [q_n, 0, 0, .1]);
%          %if(i>9)
%          scatter3(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),'*');
%          marker_id = sprintf('cg_%d',i);
%          text(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),marker_id);
%          hold on
%          %end
%  end
% title('Plot transform Frames');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% T_num = double(T_num);



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

for i=3:6
    Jw(1:3,1:6,i+3)=zeros(3,6);
    if(i<7)
    Jw(1:3,1:i,i+3)=Jw_m(1:3,1:i);
    else
    Jw(1:3,1:6,i+3)=Jw_m(1:3,1:6);
    end
end

    Jw(1:3,1:6,10)=Jw(1:3,1:6,2);
    Jw(1:3,1:6,11)=Jw(1:3,1:6,2);


%Jw_num(:,:,:)=double((subs(Jw(:,:,:),[q1 q2 q3 q4 q5 q6],q_n)));

%% Calculate End Link Jacobians
%Linear Jacobian

for i=1:length(T)
    Jv(1:3,1:6,i)=[diff(T(1:3,4,i),q1),diff(T(1:3,4,i),q2),diff(T(1:3,4,i),q3),diff(T(1:3,4,i),q4),diff(T(1:3,4,i),q5),diff(T(1:3,4,i),q6)];
end 

J_end = [Jv(:,:,11);Jw(:,:,11)]; %End Effector Jacobian
%% Calculate D Matrix 


if dynamic==1



%%
for i=1:length(Jv_cg)
   %B(1:9,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*I(1:3,1:3,i)*Jw(:,:,i);
   if (i==5)
   B(1:6,1:6,i) = zeros(6);
   elseif(i==10)
      B(1:6,1:6,i)=M(i)*Jv_cg(:,:,i)'*Jv_cg(:,:,i)+Jw(:,:,i)'*Tee_cg(1:3,1:3,1)*I(1:3,1:3,i)*Tee_cg(1:3,1:3,1)'*Jw(:,:,i);     
   elseif(i==11)
      B(1:6,1:6,i)=M(i)*Jv_cg(:,:,i)'*Jv_cg(:,:,i)+Jw(:,:,i)'*Tee_cg(1:3,1:3,2)*I(1:3,1:3,i)*Tee_cg(1:3,1:3,2)'*Jw(:,:,i);     
   else
       B(1:6,1:6,i)=M(i)*Jv_cg(:,:,i)'*Jv_cg(:,:,i)+Jw(:,:,i)'*T(1:3,1:3,i)*I(1:3,1:3,i)*T(1:3,1:3,i)'*Jw(:,:,i); %Not sure about transformations
   end 
end

D = zeros(dof);
for i=1:(dof+5)
    D = D + B(1:dof,1:dof,i);
end 


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
  P(i) = g*M(i)*p_cg(i,3);
  
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

temp=strcat(filename,'_temp_sim.mat');
save(temp);
%T=combine(T,'sincos');

%Dt=collect(Dt,[Ixx,Ixy,Ixz,Iyy,Iyz,Izz, lc1, lc2, lc3, lc4, lc5, lc6, lc7, lc8, lc9,m1, m2, m3, m4, m5, m6, m7, m8, m9]);
Dt=collect(Dt,[Ixx,Ixy,Ixz,Iyy,Iyz,Izz,reshape(l_cg,[1,numel(l_cg)]),M]);

% for i=1:length(p_cg)
%     X1_X2_mX(i) = sprintf('l%d_1_l%d_2_m%d',[i i i]);
%     X2_X3_mX(i) = sprintf('l%d_2_l%d_3_m%d',[i i i]);
%     X1_X3_mX(i) = sprintf('l%d_1_l%d_3_m%d',[i i i]);
%     X1_X1_mX(i) = sprintf('l%d_1_l%d_1_m%d',[i i i]);
% end
% 

lclcm = sym('lc%d_%d_lc%d_%d_m%d', [num 3 num 3 num]);
lclcm_sym= sym(zeros(num,3,num,3,num));

for i=1:num
    for j=1:3
        for k=1:num
            for l=1:3
                for m=1:num
                    if(k<i||l<j)
                        lclcm(i,j,k,l,m) = 0;
%                     elseif(l>j)
%                         lclcm_sym(i,j,k,l,m) = 0;
                    else
                    lclcm_sym(i,j,k,l,m)=l_cg(i,j)*l_cg(k,l)*M(m);
                    end
                end
            end
        end
    end
end

lcm = sym('lc%d_%d_m%d', [num 3 num]);
lcm_sym= sym(zeros(num,3,num));

for i=1:num
    for j=1:3
        for m=1:num
            lcm_sym(i,j,m)=l_cg(i,j)*M(m);
        end
    end
end


lclcm = reshape(lclcm,1,[]);
lclcm_sym = reshape(lclcm_sym,1,[]);

lcm = reshape(lcm,1,[]);
lcm_sym = reshape(lcm_sym,1,[]);


Dt = subs(Dt, lclcm_sym,lclcm);
Dt = subs(Dt, lcm_sym,lcm);


% lcX_2_mX = reshape(lcX_2_mX,1,[]);
% 
% lcX_mX = sym('lc%d_m%d', [9 9]);
% lcX_mX = reshape(lcX_mX,1,[]);


% for i = 1:length(M)
%     if i==1
%     LcX_mX_mult = Lc*M(i);
%     LcX2_mX_mult = (Lc.^2)*M(i);
%     else
%     LcX_mX_mult = [LcX_mX_mult, Lc*M(i)];
%     LcX2_mX_mult = [LcX2_mX_mult, (Lc.^2)*M(i)];
%     end 
%  end 

% Dt = subs(Dt, LcX2_mX_mult,lcX_2_mX);
% 
% Dt = subs(Dt, LcX_mX_mult ,lcX_mX);
temp=strcat(filename,'_temp_subs.mat');
save(temp)


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
[Ys1, Ys2, Par1, Par2, cond_ys2]=lumping_parameters_new(Y,Par);

% %% Trajectory Optimization 
% 
% [x, v, cond_save]=traj_opt_rand(Ys2,size(Ys2,2));


%Show the Results
disp('Resulting Regressor Matrix: ')
%Ys2

disp('Resulting Identifiable Parameters: ')
length(Par2)

temp=strcat(filename,'_all.mat');
save(temp)

temp2=strcat(filename,'_Y.mat');
save(temp2,'Ys2','Par2','Q','Qd','Qdd')
end