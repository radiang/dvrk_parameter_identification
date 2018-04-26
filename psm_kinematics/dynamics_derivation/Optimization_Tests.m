clear all

A=[];
b=[];
Ae=[];
be=[];
%Initial Guess
x0=[0.5 0.5 0.5 0.5];
lb=[-0.1 -0.1 -0.1 -0.1];
ub=[1 1 1 1];

%objectivefunction
syms x1 x2 x3 x4
X=[x1 x2 x3 x4];


W=sym(0);
for i=1
W(2*(i-1)+1,1:2)=[X(2*(i-1)+1) x3];
W(2*(i-1)+2,:)=[X(2*(i-1)+2) x4];
end

D=det(W);
f = matlabFunction(D,'File','tests.m','vars',{x} );


[x Fin]=fmincon(f,x0,A,b,Ae,be,lb,ub);