
function [gen, dyn, map]=psm_dynamics_f(gen)
syms T Jw lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9 l2 l3 l4 l5 l6 l7 l8 l9 m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6  qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 thet1 ps  tau1 tau2 tau3 tau4 tau5 tau6 real; 

gen.q   = [q1 q2 q3 q4 q5 q6];
gen.qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
gen.qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];

gen.Q = [q1 q2 q3 q4 q5 q6]';
gen.Qd = [qd1 qd2 qd3 qd4 qd5 qd6]';
gen.Qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6]';
gen.Tau = [tau1 tau2 tau3 tau4 tau5 tau6]';

dyn.M=[m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11];
dyn.Lc=[lc1 lc2 lc3 lc4 lc5 lc6 lc7 lc8 lc9];

%% Options
%calculate dynamics?? 
dyn.dynamic = 1;

%Gravity 
dyn.g =9.8; %m/s^2

%% Try Plotting to check transformations
%Joint Angles
q_n = [0.2, 0, .02, 0, 0, 0];  %Put your numeric values here

%Mass Locations
lc_n = [-.03, .150/2, .516/2, .2881/2, .4162/2, .001, .001, .001]; %Put numeric values of Cg locations

%% DH Paremeters and Transforms of PSM 
dyn.p = sym(pi()); 
%My own DH from RViz and DH Paper
dyn.beta=atan((.072613-.0296)/(.2961-.1524));

dyn.a      =    [ 0         ,                  0,               .150,                      .516,              .2881,     -.0430,               0,      0,     .0091,    .0102,    0];
dyn.alpha  =    [-dyn.p/2   ,           -dyn.p/2,                  0,                         0,                  0,    dyn.p/2,           0,     dyn.p/2,     -dyn.p/2,        0, -dyn.p/2];
dyn.d      =    [ 0.1524,                  .0296,                  0,                         0,                  0,          0,  .4162+gen.q(3),       0,        0,        0,    0];
dyn.tet    =    [ dyn.p/2   ,  -dyn.p/2+gen.q(1), -dyn.beta+gen.q(2), dyn.beta+dyn.p/2-gen.q(2),  -dyn.p/2+gen.q(2),   -dyn.p/2,      dyn.p/2,  dyn.p/2+gen.q(4),   dyn.p/2+gen.q(5),       gen.q(6), -dyn.p/2];

%1: Yaw Coordinate Frame
%2-4: Pitch Coordinate Frames
%  6: Insertion Coordinate Frame
%  7: Tool Roll Coordinate Frame
%  8: Tool Pitch Coordinate Frame
%  9: Tool Yaw Coordinate Frame
% 11: Tool end effector
% 12: Counterweight Pitch
% 13: Counterweight Insertion

for i=1:length(dyn.a)
dyn.Ti_i(1:4,1:4,i) = DH(dyn.tet(i),dyn.a(i),dyn.d(i),dyn.alpha(i));
%Ti_i(1:4,1:4,i) = dh_modified(tet(i),a(i),d(i),alpha(i));
end

for i=1:length(dyn.a)
    if i==1
        dyn.T(1:4,1:4,i) = dyn.Ti_i(:,:,i);
    else
    dyn.T(1:4,1:4,i)=dyn.T(1:4,1:4,i-1)*dyn.Ti_i(1:4,1:4,i);
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
dyn.Ti_counter(:,:,i) = DH(tet_c(i),a_c(i),d_c(i),alpha_c(i)); 
if i==1 
    dyn.T_c(:,:,i)=dyn.Ti_counter(:,:,i);
else 
    dyn.T_c(:,:,i)=dyn.T_c(:,:,i-1)*dyn.Ti_counter(:,:,i);
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

%T_cg 5 does not have any mass and doesnt contribute to anything

%% Syms Stuff

dyn.num = 11;

dyn.l_cg = sym('l_cg%d_%d', [dyn.num 3], 'real');
dyn.l_cg = [dyn.l_cg,ones(dyn.num,1)];
%Inertia Matrix notation
dyn.Ixx = sym('I%d_xx', [dyn.num 1],'real');
dyn.Iyy = sym('I%d_yy', [dyn.num 1],'real');
dyn.Izz = sym('I%d_zz', [dyn.num 1],'real');
dyn.Ixy = sym('I%d_xy', [dyn.num 1],'real');
dyn.Ixz = sym('I%d_xz', [dyn.num 1],'real');
dyn.Iyz = sym('I%d_yz', [dyn.num 1],'real');

for i=1:1:length(dyn.Ixx)
    
    dyn.I(1:3,1:3,i)=[dyn.Ixx(i),dyn.Ixy(i),dyn.Ixz(i); 
              dyn.Ixy(i),dyn.Iyy(i),dyn.Iyz(i);
              dyn.Ixz(i),dyn.Iyz(i),dyn.Izz(i);]; 
end

%% Inertia in Link frame

for i = 1:length(dyn.Ixx)
   S = [0, -dyn.l_cg(i,3), dyn.l_cg(i,2)   ;dyn.l_cg(i,3)  ,0 , -dyn.l_cg(i,1)   ; -dyn.l_cg(i,2)  , dyn.l_cg(i,1)    , 0  ]; 
    
dyn.L(:,:,i) = dyn.I(:,:,i) + dyn.M(i)*S'*S;

end
%% Make Stuff Zero


% M(11)=0;
% I(1:3,1:3,11) = zeros(3);
% l_cg(11,:) = zeros(1,3);

%In Plane Masses
% l_cg(1,1) = 0;
% l_cg(2,3) = 0;
% l_cg(3,3) = 0;
% l_cg(4,3) = 0;
% l_cg(6,2) = 0;
% l_cg(7,1) = 0;
% l_cg(8,3) = 0;
% l_cg(9,2) = 0;
% l_cg(11,2) = 0;
% 
% 
% l_cg(6,1)=0;
% l_cg(1,2)=0;

%Lump Gripper stuff 
dyn.M(7)=0;
dyn.I(1:3,1:3,7) = zeros(3);
dyn.l_cg(7,:) = zeros(1,4);

dyn.M(8)=0;
dyn.I(1:3,1:3,8) = zeros(3);
dyn.l_cg(8,:) = zeros(1,4);

%Lump More Stuff
% M(6)=0;
% I(1:3,1:3,6) = zeros(3);
% l_cg(6,:) = zeros(1,4);

dyn.M(9)=0;
dyn.I(1:3,1:3,9) = zeros(3);
dyn.l_cg(9,:) = zeros(1,4);


%Assume counterweight pitch goes to zero 
dyn.M(10)=0;
dyn.I(1:3,1:3,10) = zeros(3);
dyn.l_cg(10,:) = zeros(1,4);

% M(11)=0;
% I(1:3,1:3,11) = zeros(3);
% l_cg(11,:) = zeros(1,4);

%In Axis Masses
%  dyn.l_cg(1,3) = 0;
%  dyn.l_cg(1,2) = dyn.d(2)/2;
%  dyn.l_cg(2,2:3) = 0;
%  dyn.l_cg(3,2:3) = 0;
%  dyn.l_cg(4,2:3) = 0;
%  dyn.l_cg(6,1:2) = 0;
%  dyn.l_cg(7,1) = 0; 
%  dyn.l_cg(7,3) = 0; 
%  dyn.l_cg(8,2:3) = 0;
%  dyn.l_cg(9,2:3) = 0;
%  dyn.l_cg(11,1:2) = 0;


%% Let's try a new Center of Mass 

dyn.map = [q1,q2,-q2,q2,0,q3,q4,q5,q6];
coeff_travel_11= (136-15.5)/200;
% %INPLANE cg_i from frame i
for i = 1:length(dyn.l_cg)-2 
    if (i==6)
        dyn.p_cg(i,:)  = dyn.T(:,:,i)*DH(0,0,dyn.map(i),0)*transpose(dyn.l_cg(i,:));
    else
        dyn.p_cg(i,:)  = dyn.T(:,:,i)*DH(dyn.map(i),0,0,0)*transpose(dyn.l_cg(i,:));
    end 
end
%INAXIS cg_i from frame i+1
% for i = 1:length(dyn.l_cg)-2 
% dyn.p_cg(i,:)  = dyn.T(:,:,i+1)*transpose([dyn.l_cg(i,:)]);
% end 

counter_num = 2;
dyn.Tee_cg(:,:,1)= dyn.T(:,:,2)*DH(dyn.map(2),0,0,0);
%dyn.Tee_cg(:,:,2)= dyn.T(:,:,2)*DH(-dyn.p/2-dyn.beta+gen.q(2),dyn.l_cg(11,1),0,-dyn.p/2)*DH(0,0,dyn.map(6),0);
dyn.Tee_cg(:,:,2)= dyn.T(:,:,2)*DH(-dyn.p/2-dyn.beta+gen.q(2),0,0,-dyn.p/2)*DH(0,0,coeff_travel_11*dyn.map(6),0);

 dyn.p_cg(end+1,:) = dyn.Tee_cg(:,:,1)*transpose(dyn.l_cg(10,:)); %Pitch Counterweight 
 dyn.p_cg(end+1,:) = dyn.Tee_cg(:,:,2)*transpose(dyn.l_cg(11,:)); %Insertion Counterweight 

 
%% Plot Joint Angles Test
close all
q_n = [0, dyn.beta, 0.0, 0, 0, 0];  %Put your numeric values here

figure()
for i = 1:length(dyn.T)
        T_num(:,:,i)=subs(dyn.T(:,:,i),gen.q,q_n);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        line(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);  
        hold on
end

%T_num(:,:,:)=double(subs(dyn.T(:,:,:),gen.q,q_n));
a(1,:) = reshape(T_num(1,4,:),1,[]);
a(2,:) = reshape(T_num(2,4,:),1,[]);
a(3,:) = reshape(T_num(3,4,:),1,[]);
scatter3(a(1,:),a(2,:),a(3,:));
line(a(1,:),a(2,:),a(3,:));
xlim([-0.1, 1]);
zlim([-0.1, 1]);
ylim([-0.55,0.55]);
hold on

% 
% for i = 1:length(T_cg)
%         T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
%         
%         scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
%         marker_id = sprintf('cg_%d',i);
%         text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
%         hold on
% end

 for i = 1:length(dyn.p_cg)
         p_cg_num(i,:)=subs(dyn.p_cg(i,:),[gen.q, dyn.l_cg(i,1:3)], [q_n, 0, 0, 0.1]);
         %if(i>9)
         scatter3(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),'*');
         marker_id = sprintf('cg_%d',i);
         text(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),marker_id);
         hold on
         %end
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

 for i=1:length(dyn.p_cg)
     dyn.Jv_cg(1:3,1:6,i)=[diff(transpose(dyn.p_cg(i,1:3)),gen.q(1)),diff(transpose(dyn.p_cg(i,1:3)),gen.q(2)),diff(transpose(dyn.p_cg(i,1:3)),gen.q(3)),diff(transpose(dyn.p_cg(i,1:3)),gen.q(4)),diff(transpose(dyn.p_cg(i,1:3)),gen.q(5)),diff(transpose(dyn.p_cg(i,1:3)),gen.q(6))];
 end 

%Angular Jacobian
k=[0;0;1];
dyn.Jw_m(1:3,:)=[dyn.T(1:3,1:3,1)*k, dyn.T(1:3,1:3,4)*k,0 ,dyn.T(1:3,1:3,7)*k,dyn.T(1:3,1:3,8)*k,dyn.T(1:3,1:3,9)*k];

dyn.Jw(1:3,1:6,1)=sym(zeros(3,6));
dyn.Jw(1:3,1:1,1)=dyn.Jw_m(1:3,1);

%Close Kinematic Chain Links
dyn.Jw(1:3,1:6,2)=sym(zeros(3,6));
dyn.Jw(1:3,1:2,2)=[dyn.T(1:3,1:3,1)*k, dyn.T(1:3,1:3,2)*k];

dyn.Jw(1:3,1:6,3)=sym(zeros(3,6));
dyn.Jw(1:3,1:2,3)=[dyn.T(1:3,1:3,1)*k, -dyn.T(1:3,1:3,3)*k];

dyn.Jw(1:3,1:6,4)=sym(zeros(3,6));
dyn.Jw(1:3,1:2,4)=[dyn.T(1:3,1:3,1)*k, dyn.T(1:3,1:3,4)*k];

dyn.Jw(1:3,1:6,5)=sym(zeros(3,6));
dyn.Jw(1:3,1:2,5)=[dyn.T(1:3,1:3,1)*k, dyn.T(1:3,1:3,5)*k];

for i=3:6
    dyn.Jw(1:3,1:6,i+3)=sym(zeros(3,6));
    if(i<7)
    dyn.Jw(1:3,1:i,i+3)=dyn.Jw_m(1:3,1:i);
    else
    dyn.Jw(1:3,1:6,i+3)=dyn.Jw_m(1:3,1:6);
    end
end

    dyn.Jw(1:3,1:6,10)=dyn.Jw(1:3,1:6,2);
    dyn.Jw(1:3,1:6,11)=dyn.Jw(1:3,1:6,2);


%Jw_num(:,:,:)=double((subs(Jw(:,:,:),[q1 q2 q3 q4 q5 q6],q_n)));

%% Calculate End Link Jacobians
%Linear Jacobian

for i=1:length(dyn.T)
    dyn.Jv(1:3,1:6,i)=[diff(dyn.T(1:3,4,i),gen.q(1)),diff(dyn.T(1:3,4,i),gen.q(2)),diff(dyn.T(1:3,4,i),gen.q(3)),diff(dyn.T(1:3,4,i),gen.q(4)),diff(dyn.T(1:3,4,i),gen.q(5)),diff(dyn.T(1:3,4,i),gen.q(6))];
end 

gen.J_end = [dyn.Jv(:,:,11);dyn.Jw(:,:,11)]; %End Effector Jacobian
%% Calculate D Matrix 


if dyn.dynamic==1



%%
for i=1:length(dyn.Jv_cg)
   %B(1:9,1:6,i)=M(i)*Jv(:,:,i)'*Jv(:,:,i)+Jw(:,:,i)'*I(1:3,1:3,i)*Jw(:,:,i);
   if (i==5)
   dyn.B(1:6,1:6,i) = zeros(6);
   elseif(i==10)
      dyn.B(1:6,1:6,i)=dyn.M(i)*dyn.Jv_cg(:,:,i)'*dyn.Jv_cg(:,:,i)+dyn.Jw(:,:,i)'*dyn.Tee_cg(1:3,1:3,1)*dyn.I(1:3,1:3,i)*dyn.Tee_cg(1:3,1:3,1)'*dyn.Jw(:,:,i);     
   elseif(i==11)
      dyn.B(1:6,1:6,i)=dyn.M(i)*dyn.Jv_cg(:,:,i)'*dyn.Jv_cg(:,:,i)+dyn.Jw(:,:,i)'*dyn.Tee_cg(1:3,1:3,2)*dyn.I(1:3,1:3,i)*dyn.Tee_cg(1:3,1:3,2)'*dyn.Jw(:,:,i);     
   else
       dyn.B(1:6,1:6,i)=dyn.M(i)*dyn.Jv_cg(:,:,i)'*dyn.Jv_cg(:,:,i)+dyn.Jw(:,:,i)'*dyn.T(1:3,1:3,i)*dyn.I(1:3,1:3,i)*dyn.T(1:3,1:3,i)'*dyn.Jw(:,:,i); %Not sure about transformations
   end 
end

dyn.D = zeros(gen.dof);
for i=1:(11)
    %if 6<i&&i<10 %%%DONT FORGET THIS RADIAN, turning off wrist dynamics
     %   boo=0;
    %else
    dyn.D = dyn.D + dyn.B(1:gen.dof,1:gen.dof,i);
    %end
end 


%D=combine(D,'sincos');
dyn.D = simplify(dyn.D);
dyn.D = combine(dyn.D);

%% Crystoffel Symbols
for i=1:gen.dof
    for j=1:gen.dof
        for k=1:gen.dof
            dyn.c(i,j,k)=diff(dyn.D(k,j),gen.Q(i))+diff(dyn.D(k,i),gen.Q(j))-diff(dyn.D(i,j),gen.Q(k));
        end
    end
end

dyn.C=sym(zeros(gen.dof,gen.dof));
    for j=1:gen.dof
        for k=1:gen.dof
            for i=1:gen.dof
                dyn.C(k,j)= dyn.C(k,j) + dyn.c(i,j,k)*gen.Qd(i);
                %C(k,j)=(c(1,j,k)*Qd(1)+c(2,j,k)*Qd(2)+c(3,j,k)*Qd(3)+c(4,j,k)*Qd(4)+c(5,j,k)*Qd(5)+c(6,j,k)*Qd(6));
            end
        end
    end
    
%%  Potential Energy
  dyn.P=sym(zeros(1,length(dyn.p_cg)));
  dyn.P_tog=sym(zeros(1));
  dyn.Psi = sym(zeros(gen.dof,1));
  
  for i=1:length(dyn.p_cg)
  dyn.P(i) = dyn.g*dyn.M(i)*dyn.p_cg(i,3);
  
  if (i==5)
      dyn.P(i)=0;
  end
  
  dyn.P_tog = dyn.P_tog + dyn.P(i);
  
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
  
  
for i=1:gen.dof 
dyn.Psi(i,1)=diff(dyn.P_tog,gen.Q(i));
end

%% Put together
dyn.Dt=dyn.D(1:gen.dof,1:gen.dof)*gen.Qdd(1:gen.dof)+dyn.C(1:gen.dof,1:gen.dof)*gen.Qd(1:gen.dof)+dyn.Psi(1:gen.dof,:);

%% Frictiones 
dyn.Fv=sym('Fv_%d',[6 1],'real');
dyn.Fvl=sym('Fvl_%d%d',[2 2],'real');

dyn.M_Fv = diag(dyn.Fv);
dyn.M_Fv(5:6,5:6)=dyn.Fvl;

dyn.Fs = sym('Fs_%d',[6 1],'real');
dyn.M_Fs = diag(dyn.Fs);

dyn.Ke = sym('Ke_%d',[6 1], 'real');

dyn.M_Ke = diag(dyn.Ke);
dyn.M_Ke(3,3)= 0;
dyn.M_Ke(5,5)= 0;
dyn.M_Ke(6,6)= 0;

dyn.Dt=dyn.Dt-dyn.M_Fv(1:gen.dof,1:gen.dof)*gen.Qd(1:gen.dof)-dyn.M_Fs(1:gen.dof,1:gen.dof)*sign(gen.Qd(1:gen.dof))-dyn.M_Ke(1:gen.dof,1:gen.dof)*gen.Q(1:gen.dof);
dyn.Dt=simplify(dyn.Dt);


%% 

temp=strcat(gen.filename,'_temp_sim.mat');
save(temp);
%T=combine(T,'sincos');

%Dt=collect(Dt,[Ixx,Ixy,Ixz,Iyy,Iyz,Izz, lc1, lc2, lc3, lc4, lc5, lc6, lc7, lc8, lc9,m1, m2, m3, m4, m5, m6, m7, m8, m9]);
dyn.Dt=collect(dyn.Dt,[dyn.Ixx,dyn.Ixy,dyn.Ixz,dyn.Iyy,dyn.Iyz,dyn.Izz,reshape(dyn.l_cg,[1,numel(dyn.l_cg)]),dyn.M,dyn.Fv,reshape(dyn.Fvl,1,[]),dyn.Fs,dyn.Ke]);

% for i=1:length(p_cg)
%     X1_X2_mX(i) = sprintf('l%d_1_l%d_2_m%d',[i i i]);
%     X2_X3_mX(i) = sprintf('l%d_2_l%d_3_m%d',[i i i]);
%     X1_X3_mX(i) = sprintf('l%d_1_l%d_3_m%d',[i i i]);
%     X1_X1_mX(i) = sprintf('l%d_1_l%d_1_m%d',[i i i]);
% end
% 

dyn.lclcm = sym('lc%d_%d_lc%d_%d_m%d', [dyn.num 3 dyn.num 3 dyn.num]);
dyn.lclcm_sym= sym(zeros(dyn.num,3,dyn.num,3,dyn.num));

for i=1:dyn.num
    for j=1:3
        for k=1:dyn.num
            for l=1:3
                for m=1:dyn.num
                    if(k<i||l<j)
                        dyn.lclcm(i,j,k,l,m) = 0;
%                     elseif(l>j)
%                         lclcm_sym(i,j,k,l,m) = 0;
                    else
                    dyn.lclcm_sym(i,j,k,l,m)=dyn.l_cg(i,j)*dyn.l_cg(k,l)*dyn.M(m);
                    end
                end
            end
        end
    end
end

dyn.lcm = sym('lc%d_%d_m%d', [dyn.num 3 dyn.num]);
dyn.lcm_sym= sym(zeros(dyn.num,3,dyn.num));

for i=1:dyn.num
    for j=1:3
        for m=1:dyn.num
            dyn.lcm_sym(i,j,m)=dyn.l_cg(i,j)*dyn.M(m);
        end
    end
end


dyn.lclcm = reshape(dyn.lclcm,1,[]);
dyn.lclcm_sym = reshape(dyn.lclcm_sym,1,[]);

dyn.lcm = reshape(dyn.lcm,1,[]);
dyn.lcm_sym = reshape(dyn.lcm_sym,1,[]);


dyn.Dt = subs(dyn.Dt, dyn.lclcm_sym,dyn.lclcm);
dyn.Dt = subs(dyn.Dt, dyn.lcm_sym,dyn.lcm);


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
temp=strcat(gen.filename,'_temp_subs.mat');
save(temp)

if gen.dof<6
dyn.Dt = subs(dyn.Dt, gen.q(gen.dof+1:6),zeros(1,6-gen.dof));
end

%% Regressor Matrix Form
gen.Par=symvar(dyn.Dt);

%Delete q's from Par
check=gen.q;
check(end+1:end+length(gen.qd)) = gen.qd;
check(end+1:end+length(gen.qdd)) = gen.qdd;
[bullshit,ind]=ismember(check,gen.Par);

 for k=length(ind):-1:1
       if ind(k)~= 0
       gen.Par(ind(k))=[];
       end
 end
 
 
%Par(end-17:end)=[]; 
[gen.Y, tau]=equationsToMatrix(dyn.Dt == gen.Tau(1:gen.dof), gen.Par);

%% Stribeck Friction for Joint 3
syms Fc_3  delta_s  vs


m = find(gen.Par == 'Fs_3');
temp_Fc = sign(qd3)*(exp(-(abs(qd3)/abs(vs))^delta_s) - 1); 
temp_Fs = -exp(-(abs(qd3)/abs(vs))^delta_s)*sign(qd3);

%Fs
gen.Y(:,m) = [0, 0, temp_Fs].';

%Fc
temp = [gen.Y(:,1:m), [0,0, temp_Fc].', gen.Y(:,m+1:end)];
gen.Y(:,:)=[];
gen.Y=temp;
gen.Par = [gen.Par(1,1:m), Fc_3, gen.Par(1,(m+1):end)];

%Offsets
m = find (gen.Par == 'Fc_3');
temp = [gen.Y(:,1:m), [0,0, 1].', gen.Y(:,m+1:end)];
gen.Y(:,:)=[];
gen.Y=temp;

syms Fco_3 Fso_3
gen.Par = [gen.Par(1,1:m), Fco_3+Fso_3, gen.Par(1,(m+1):end)];

% Valus of vs and delta s
delta_n = 0.6;
vs_n = 0.0055;

gen.Y = subs(gen.Y, [vs, delta_s], [vs_n, delta_n]);


%% Lumping Parameters
cond_ys2_lump=100000;
 old = cond_ys2_lump;
 counter = 1;
while cond_ys2_lump>200 && counter<40
    

%finding the linear combinations
%[Ys1, Ys2, Par1, Par2, cond_ys2, W]=lumping_parameters_new(Y,Par,dof);
[Ys2,Par2,cond_ys2_lump,W,temp_map]=svd_reduction(gen.Y,gen.Par,gen.dof);
% %% Trajectory Optimization 
% 
% [x, v, cond_save]=traj_opt_rand(Ys2,size(Ys2,2));


%Show the Results
disp('Resulting Regressor Matrix: ')
%Ys2

disp('Resulting Identifiable Parameters: ')
length(Par2)

counter=counter+1;

if old>cond_ys2_lump
% temp=strcat('data/',gen.filename,'/all.mat');
% save(temp)
% 
% temp2=strcat('data/',gen.filename,'/Y.mat');
% save(temp2,'gen')


old = cond_ys2_lump;
 
gen.cond_ys2 = cond_ys2_lump;
gen.Par2 = Par2;
gen.Ys2 = Ys2;
map = temp_map;

end

end



end

end