%% Test Matrices

qn  = [0, 0.0, 0];
qdn = [0, 0, -0.02];
disp('-------------------------start----------------------------')

nN = double(subs(ctrl.Nu3, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]))
nC = double(subs(ctrl.C, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]))
nF = double(subs(ctrl.Fr, [gen.q(1:3), gen.qd(1:3)], [qn, qdn]))
nG = double(subs(ctrl.G, gen.q(1:3),qn))

all = nC+nF+nG

compare = nF+nN
% syms r s t
% A = [s-2*t+r^2
%         3*s-t+2*r];
% eqns = A==0;
% vars = [s t];
% [A,b] = equationsToMatrix(eqns,vars)