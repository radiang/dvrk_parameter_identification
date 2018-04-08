function [Cond] = Cond_Matrix(x,g)

%syms x(1) x(2) x(3) x(4)

% x= sym(zeros(1,4));

for t=1:4
a = sprintf('x(%d)',t);
b = sprintf('array(%d)',t);
eval(strcat(b,'=',a));
end 

A = g*[array(1) ,array(2); array(3) ,array(4)];

Cond = double(cond(A));

end