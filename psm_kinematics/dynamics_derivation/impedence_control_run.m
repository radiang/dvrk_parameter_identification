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
q_n = [0, p/4, .1, 0, 0, 0];

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
xlabel('x');
ylabel('y');
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
%% Substitute to continuous time
D_t = subs(D, [q, qd, qdd], [qt, qdt, qddt]);
C_t = subs(C, [q, qd, qdd], [qt, qdt, qddt]);
Psi_t = subs(Psi, [q, qd, qdd], [qt, qdt, qddt]);


%Fill up Inertias and Masses (From Solidworks)
m_num = 0.5*ones(1,11);
lc_num = 0.02 *ones(1,9);
Ic_num = 0.01 * ones(1,11*6);
l_cg_num = 0.02*ones(1,numel(l_cg(:,1:3)));

%eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);
%temp = symvar(eqn);

%% Actual values of the PSM from Solidworks
psm.m = [1.47048, 0.995109+0.0744253, 0.178407+0.445695, 2.09102, 0, 0.0311806 + 0.193728, 0.000342894,0.000249121,6.63661e-06, 0.995109+0.0744253, 0.0311806 + 0.193728 ]; %Kg
psm.I(:,:,1) = [ 0.01571450, 0.0000043 , 0.00000728; 0, 0.01814321, -0.00048589; 0, 0, 0.00824930];  %kg m^2 
psm.I(:,:,2) = [ 0.00202417+0.00000876, 0.00047511 , 0.00000043-0.00001711; 0, 0.00564963+0.00022654, 0.00000129; 0, 0, 0.00564716+0.00022322];  %pitch back and front added together
psm.I(:,:,3) = [ 0.00463460+0.01127103, 0.00001128-0.00031801 , 0.00000003; 0,0.00014989+0.00045979, -0.00000040; 0, 0, 0.00449516+0.01110239];  %pitch top and bottom added together
psm.I(:,:,4) = [ 0.00156905, -0.00123970, 0.00000002; 0, 0.04284466, 0.0000; 0, 0, 0.04227453];  %kg m^2 
psm.I(:,:,5) = zeros(3);  
psm.I(:,:,6) = [ 0.00006363 + 0.00139739, 0.00000014, -0.00000018-0.00000957; 0, 000004346+0.00135923, 0.00000157; 0, 0, 0.00002063+0.00006656];  %kg m^2 
psm.I(:,:,7) = zeros(3);  %kg m^2 
psm.I(:,:,8) = zeros(3);  %kg m^2 
psm.I(:,:,9) = zeros(3);  %kg m^2 
psm.I(:,:,10) = psm.I(:,:,2); %Asumptions
psm.I(:,:,11) = psm.I(:,:,6); %Asumptions

%Still assumption
psm.l_cg(1,:) = [   0   ,  0.038769    ,0.158996 ]; %x -y -z
psm.l_cg(2,:) = [   -0.012175*cos(beta)-0.036897*sin(beta)  ,  +0.012175*sin(beta)-0.036897*cos(beta)    ,0]; %
psm.l_cg(3,:) = [   0.010348*cos(beta)+0.256832*sin(beta)  ,-0.010348*sin(beta)+0.256832*cos(beta), 0 ]; %
psm.l_cg(4,:) = [ -0.001929  ,-0.136127 , 0 ]; %
psm.l_cg(5,:) = [   0  , 0 , 0 ]; %
psm.l_cg(6,:) = [   0.001528 ,-0.000127 , 0.013848]; %

psm.l_cg(11,:) = [   0 ,0 , 0]; %

%Make a Symmetric Matrix 
% psm.I=(psm.I + psm.I.')/2;

for i = 1:length(psm.I)
psm.Ixx(i)=psm.I(1,1,i);
psm.Ixy(i)=psm.I(1,2,i);
psm.Ixz(i)=psm.I(1,3,i);
psm.Iyy(i)=psm.I(2,2,i);
psm.Iyz(i)=psm.I(2,3,i);
psm.Izz(i)=psm.I(3,3,i);
end

%% Substitution
D_t = subs(D_t, [ reshape(l_cg(:,1:3),1,[]),transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz), M ] ,[reshape(psm.l_cg(:,1:3),1,[]),psm.Ixx,psm.Ixy,psm.Ixz,psm.Iyy,psm.Iyz,psm.Izz, psm.m]);
C_t = subs(C_t, [ reshape(l_cg(:,1:3),1,[]),transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz), M ] ,[reshape(psm.l_cg(:,1:3),1,[]),psm.Ixx,psm.Ixy,psm.Ixz,psm.Iyy,psm.Iyz,psm.Izz, psm.m]);
Psi_t = subs(Psi_t, [ reshape(l_cg(:,1:3),1,[]),transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz), M ] ,[reshape(psm.l_cg(:,1:3),1,[]),psm.Ixx,psm.Ixy,psm.Ixz,psm.Iyy,psm.Iyz,psm.Izz, psm.m]);

%eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);

eqn = D_t(1:3,1:3) * transpose(qddt(1:3));

ctrl.xe = subs(T(1:3,4,11), q, qt);
ctrl.xe_d = diff(ctrl.xe,t);
ctrl.xe_dd= diff(ctrl.xe,t,t);


%% Force Controller in x || R^3
ctrl.Ja=J_end_t(1:3,1:3);
ctrl.Jd=J_diff_t(1:3,1:3);
%Set up Environment
%Initial Condition
joint.qt = qt;
joint.qdt = qdt;
joint.qddt = qddt;

ctrl.D_t=D_t;

%Elastic Modulus of Object [N/m]
ctrl.K = 10000; 

% Set up Controllers
ctrl.Md = eye(3);
ctrl.Kd = 30*eye(3);
ctrl.Kp = 100*eye(3);

%Elasticity of manipulator;


%Initial Conditions
qd0 = [0.0001 ;0.00001 ;0.00001;0;0;0];
q0 = [0 ;0 ;0.1;0;0;0];

init.x0n = double(subs(T(1:3,4,11), q, q0.'));

%Desired Set Position
syms xd fd
ctrl.X0 = sym('x0_%d',[3 1]);
ctrl.Fd = sym('Fd_%d',[3 1]);
ctrl.Xd = sym('Xd_%d',[3 1]);

%Set Desired Position (Cartesian)
init.xd = [0.01 0.1 0.01].'+init.x0n;
init.xd_d = [0 0 0].';
init.xd_dd = [0 0 0].';


%elastic forces in object 
fe = sym(zeros(3,1));
%fe(1) = ctrl.K*(ctrl.xe(1) - init.x0n(1));
ctrl.he = simplify(fe);
ctrl.ha=ctrl.he; %assume geo Jac same as ana Jac


% Impedence Control with desired force tracking
u = impedence_control(ctrl,joint,init);

save('temp2.mat');

%% other
if (debug==0)
    
%% Make ODE Function 

dyn_eqn=u-eqn;

dyn_eqn=simplify(dyn_eqn);
%dyn_eqn = eqn==u;

[eqs,vars] = reduceDifferentialOrder(dyn_eqn==0,[q1t,q2t,q3t]);
[Mass,F] = massMatrixForm(eqs,vars);

% 
% F = vpa(F,2);
% M = vpa(M,2);

F=simplify(F);
Mass=simplify(Mass);

%f = M\F; too slow doesnt work

Mfun = odeFunction(Mass,vars,'File','myfileM','Comments','Version: 1.1');
Ffun = odeFunction(F,vars,'File','myfileF','Comments','Version: 1.1');
opt = odeset('mass', Mfun, 'InitialSlope', qd0(1:3));

[time, q_out] = ode45(Ffun, [0 7], [q0(1:3);qd0(1:3)], opt);


%% Plot Output
beg = 1;
ends = length(q_out);

T_out = subs(T(1:3,4,11),[q4 q5 q6],[0 0 0]);

for i = beg:ends
        %output(1:3,i)=subs(T(1:3,4,11),[q1 q2 q3 q4 q5 q6],[q_out(i,1:3) 0 0 0]);
        q1 = q_out(i,1);
        q2 = q_out(i,2);
        q3 = q_out(i,3);
        
        output(1,i)=               0.15*sin(q2 - 0.29) + 0.043*cos(q2) - 0.15*sin(q2) - 1.0*q3*sin(q2) + 0.49;
        output(2,i)=-2.0e-4*sin(q1)*(744.0*cos(q2) - 755.0*cos(q2 - 0.29) + 222.0*sin(q2) + 5.0e3*q3*cos(q2));
        output(3,i)=          1.3e-5*cos(q1)*sin(q2) - 3.7e-3*cos(q1)*cos(q2) - 1.0*q3*cos(q1)*cos(q2) + 0.15;
 
%         scatter3(output(1),output(2),output(3));
%         marker_id = sprintf('%d',i);
        %text(output(1),output(2),output(3),marker_id);     
        hold on
end


figure()
        %scatter3(output(1,beg:ends),output(2,beg:ends),output(3,beg:ends));
        line(output(1,beg:ends),output(2,beg:ends),output(3,beg:ends));
        hold on
%         
        scatter3(output(1,beg),output(2,beg),output(3,beg),'r');     
        hold on
%         output(1:3)=subs(T(1:3,4,11),[q1 q2 q3 q4 q5 q6],[q_out(end,1:3) 0 0 0]);
%         scatter3(output(1),output(2),output(3));
%         marker_id = sprintf('%d',i);
%         text(output(1),output(2),output(3),marker_id);     
%         hold on

title('Plot PSM Force output');
xlabel('x');
ylabel('y');
zlabel('z');

end
