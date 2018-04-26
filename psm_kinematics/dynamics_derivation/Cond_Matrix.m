function [Cond] = Cond_Matrix(x)

load('W.mat');
y = symvar(W);
W = subs(W, [y(1) y(2)],[x(1) x(2)]);

Cond = double(cond(W));

end