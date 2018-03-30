clear all 
close all

load('psm_new_dynamics.mat');

syms  q1t(t) q2t(t) q3t(t) q4t(t) q5t(t) q6t(t)

qt = [q1t(t) q2t(t) q3t(t) 0 0 0];
qdt = diff(qt,t);
qddt = diff(qdt,t);

%% Jacobian End Effector and it's Derivative
J_end_t = subs(J_end, symvar(J_end), [q1t(t) q2t(t) q3t(t) 0 0 0]);
J_diff_t = diff(J_end_t,t);


%% Substitute to continuous time
D_t = subs(D, [q, qd, qdd], [qt, qdt, qddt]);
C_t = subs(C, [q, qd, qdd], [qt, qdt, qddt]);
Psi_t = subs(Psi, [q, qd, qdd], [qt, qdt, qddt]);


%Fill up Inertias and Masses (From Solidworks)

m_num = 0.5*ones(1,9);
lc_num = 0.01 *ones(1,9);
Ic_num = 0.01 * ones(1,9*6);

eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);
temp = symvar(eqn);

D_t = subs(D_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);
C_t = subs(C_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);
Psi_t = subs(Psi_t, [transpose(Ixx), transpose(Ixy), transpose(Ixz), transpose(Iyy), transpose(Iyz), transpose(Izz),Lc, M ] ,[Ic_num, lc_num, m_num]);

%eqn = D_t(1:3,1:3) * qddt(1:3)' + C_t(1:3,1:3)*qdt(1:3)'+ Psi_t(1:3);

eqn = D_t(1:3,1:3) * qddt(1:3)';

xe = subs(T(1:3,4,11), q, qt);
xe_d = diff(xe,t);


%% Force Controller in x || R^3


%Set up Environment
%Initial Condition
x0 = double(subs(T(1:3,4,11), q, [0 0 0 0 0 0]));


%Elastic Modulus of Object [N/m]
K = 10000; 

%Desired Set Position
xd = [0.1 0.1 0.1]'-x0;
fd = [10, 0, 0]';

% Set up Controllers
Md = eye(3);
Kd = eye(3);
Kp = eye(3);
Cf = eye(3);
Ci = eye(3);

%elastic forces in object 
fe = K*(xe(1) - x0(1));
he=0; %<This is also elastic force weird mann
%equation 9.30 


xf=Cf*(fd-fe);

y = inv(J_end_t(1:3,1:3)) * inv(Md)*(-Kd*xe_d + Kp *(xd-xe + xf) - Md*J_diff_t(1:3,1:3)*qdt(1:3)');
u = D_t(1:3,1:3) * y + J_end_t(1:3,1:3)'*fe;



%% Make ODE Function 
dyn_eqn = eqn==u;

[eqs,vars] = reduceDifferentialOrder(dyn_eqn,[q1t,q2t,q3t]);
[M,F] = massMatrixForm(eqs,vars);


F = vpa(F,2);
M = vpa(M,2);

%F=simplify(F);
%f = M\F;

M = odeFunction(M,vars,'File','myfileM','Comments','Version: 1.1');
F = odeFunction(F,vars,'File','myfileF','Comments','Version: 1.1');

qd0 = [0 ;0 ;0;0;0;0];
q0 = [0 ;0 ;0;0;0;0];

opt = odeset('mass', M, 'InitialSlope', qd0);


ode15s(F, [0 7], q0, opt)

