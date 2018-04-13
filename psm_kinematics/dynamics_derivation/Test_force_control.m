clear all 
close all

%% 3DOF FORCE Controller 
filename='3dof_inplanepitch_svd';
loadname = strcat('data/',filename,'_all.mat');
load(loadname);

syms  q1t(t) q2t(t) q3t(t) q4t(t) q5t(t) q6t(t)

qt = [q1t(t) q2t(t) q3t(t) 0 0 0];
qdt = diff(qt,t);
qddt = diff(qdt,t);

%% Options
debug = 0;

%% Jacobian End Effector and it's Derivative
J_end_t = subs(J_end, symvar(J_end), qt);
J_diff_t = diff(J_end_t,t);

%Make 3dof Jacobian 
J3 = subs(J_end(1:6,1:3),[q4, q5, q6],[0 0 0]);

J3_diff = subs(J_diff_t(1:6,1:3), [diff(q1t,t),diff(q2t,t),diff(q3t,t)],transpose(Qd(1:3)));
J3_diff = subs(J3_diff, q1t, Q(1));
J3_diff = subs(J3_diff, q2t, Q(2));
J3_diff = subs(J3_diff, q3t, Q(3));


%% Check Jacobian in plot 
q_n = [0, 0, 0, 0, 0, 0];

J3_num = subs(J_end_t(1:6,1:3),q1t,q_n(1));
J3_num = subs(J3_num,q2t,q_n(2));
J3_num = subs(J3_num,q3t,q_n(3));

J3_num = double(J3_num);

figure()
% for i = 1:length(T)
%         T_num(:,:,i)=subs(T(:,:,i),[q1 q2 q3 q4 q5 q6],q_n);
%         
%         scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
%         line(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
%         marker_id = sprintf('%d',i);
%         text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);     
%         hold on
% end


T_num(:,:,:)=double(subs(T(:,:,:),[q1 q2 q3 q4 q5 q6],q_n));
a(1,:) = reshape(T_num(1,4,:),1,[]);
a(2,:) = reshape(T_num(2,4,:),1,[]);
a(3,:) = reshape(T_num(3,4,:),1,[]);
scatter3(a(1,:),a(2,:),a(3,:));
line(a(1,:),a(2,:),a(3,:));
hold on

x0 = double(subs(T(1:3,4,11), q, q_n));

xv=J3_num*transpose([10 0 0]);
xvx=xv
xd = x0+xv(1:3);
plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'r');
hold on
xv=J3_num*transpose([0 10 0]);
xvy=xv
xd = x0+xv(1:3);
plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'g');
xv=J3_num*transpose([0 0 0.5]);
xd = x0+xv(1:3);
plot3([xd(1),T_num(1,4,11)],[xd(2),T_num(2,4,11)],[xd(3),T_num(3,4,11)],'b');


title('Plot transform Frames');
xlabel('y');
ylabel('x');
zlabel('z');
s=1;
xlim([-s/2 s])
ylim([-s/2 s])
zlim([-s/2 s])
% 
% for i = 1:length(T_cg)
%         T_cg_num(:,:,i)=subs(T_cg(:,:,i),[q1 q2 q3 q4 q5 q6 lc1 lc2 lc3 lc4 lc6 lc7 lc8 lc9], [q_n lc_n]);
%         
%         scatter3(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),'*');
%         marker_id = sprintf('cg_%d',i);
%         text(T_cg_num(1,4,i),T_cg_num(2,4,i),T_cg_num(3,4,i),marker_id);
%         hold on
% end

%  for i = 1:length(p_cg)
%          p_cg_num(i,:)=subs(p_cg(i,:),[q l_cg(i,:)], [q_n, 0, 0, .1]);
%          %if(i>9)
%          scatter3(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),'*');
%          marker_id = sprintf('cg_%d',i);
%          text(p_cg_num(i,1),p_cg_num(i,2),p_cg_num(i,3),marker_id);
%          hold on
%          %end
%  end



%% Seperate Variables to Par2 Force Controller 

[Mt, Nu, can]=seperate_f(Ys2,Par2,Qdd);

%% Make into 3 DOF
Mt3 = subs(Mt(1:3,1:3),[transpose(Q(4:6))], [0 0 0]);
Nu3 = subs(Nu(1:3,1),[transpose(Q(4:6)),transpose(Qd(4:6))],[0, 0, 0, 0, 0, 0]);
%% Generate Ccode
% 
% stringname = strcat('ccode/',filename,'_J_ccode.c');
% ccode(J3,'File',stringname,'Comments','V1.2');
% 
% stringname = strcat('ccode/',filename,'_Jd_ccode.c');
% ccode(J3_diff,'File',stringname,'Comments','V1.2');
% 
% stringname = strcat('ccode/',filename,'_Mt_ccode.c');
% ccode(Mt3,'File',stringname,'Comments','V1.2');
% 
% stringname = strcat('ccode/',filename,'_Nu_ccode.c');
% ccode(Nu3,'File',stringname,'Comments','V1.2');
% 
% stringname = strcat('ccode/',filename,'_Ys2_ccode.c');
% ccode(Ys2,'File',stringname,'Comments','V1.2');



if (debug==0)
%% Substitute to continuous time
D_t = subs(D, [q, qd, qdd], [qt, qdt, qddt]);
C_t = subs(C, [q, qd, qdd], [qt, qdt, qddt]);
Psi_t = subs(Psi, [q, qd, qdd], [qt, qdt, qddt]);


%Fill up Inertias and Masses (From Solidworks)
m_num = 0.5*ones(1,11);
lc_num = 0.01 *ones(1,9);
Ic_num = 0.01 * ones(1,11*6);
l_cg_num = 0.02*ones(1,numel(l_cg));
% 
% eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);
% temp = symvar(eqn);
% 
% D_t = subs(D_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);
% C_t = subs(C_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);
% Psi_t = subs(Psi_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);
% 
% %eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);
% 
% eqn = D_t(1:3,1:3) * qddt(1:3)';

D_m = subs(D, [reshape(l_cg,1,[]),transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[l_cg_num,Ic_num, lc_num, m_num]);



%% Force Controller in x || R^3
clear saveu
clear save

%Set up Environment
%Initial Condition
x0 = double(subs(T(1:3,4,11), q, [0 0 0.1 0 0 0]));
xe_d0=[0, 0 , 0].';

%Elastic Modulus of Object [N/m]
K = 10000; 



% Set up Controllers
Md = eye(3);
Kd = eye(3);
Kp = eye(3);
Cf = eye(3);
Ci = eye(3);

%elastic forces in object 

he=0; %<This is also elastic force weird mann
%equation 9.30 




% y = inv(J_end_t(1:3,1:3)) * inv(Md)*(-Kd*xe_d + Kp *(xd-xe + xf) - Md*J_diff_t(1:3,1:3)*qdt(1:3)');
% u = D_t(1:3,1:3) * y + J_end_t(1:3,1:3)'*fe;




xe = x0;
qn = [0,0,0].';
qnv = [0,0,0].';
xe_d = subs(J3(1:3,1:3),symvar(J3),qn.')*qn;
fe = [0,0,0].';


%Desired Set Position
xd = [0.00 0.01 0.00]'+x0;
fd = [0, 0, 0]';
save(:,1)=x0;
saveu(:,1)=zeros(3,1);
%% Test Loop
for i=1:20

%fe(1) = K*(xe(1) - x0(1));
xf=Cf*(fd-fe);
y = inv(J3(1:3,1:3)) * inv(Md)*(-Kd*xe_d + Kp *(xd-xe + xf) - Md*J3_diff(1:3,1:3)*qnv(1:3));
u = D_m(1:3,1:3) * y + J3(1:3,1:3)'*fe;

u = double(subs(u,[q(1:3) qd(1:3)], [qn.' qnv.']));

scale = 0.01;
qn=scale*u+qn;
qnv=scale/4*u+qnv;

xe = double(subs(T(1:3,4,11), q, [qn.' 0 0 0]));
xe_d = double(subs(J3(1:3,1:3),symvar(J3),qn.')*qn);

save(:,end+1)=xe;
saveu(:,end+1)=u;
end


%ode15s(F, [0 7], q0, opt)
figure()
 for i = 1:length(save)
        scatter3(save(1,i),save(2,i),save(3,i));
        marker_id = sprintf('%d',i);
        text(save(1,i),save(2,i),save(3,i),marker_id);     
        hold on
 end

title('Plot xe');
xlabel('x');
ylabel('y');
zlabel('z');
hold off
%ode15s(F, [0 7], q0, opt)
figure()
 for i = 1:length(saveu)
        scatter3(saveu(1,i),saveu(2,i),saveu(3,i));
        marker_id = sprintf('%d',i);
        text(saveu(1,i),saveu(2,i),saveu(3,i),marker_id);     
        hold on
 end

title('Plot u');
xlabel('x');
ylabel('y');
zlabel('z');


end
