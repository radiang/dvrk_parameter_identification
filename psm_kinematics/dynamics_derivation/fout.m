clear all

syms x

y=log(1-(1+exp(x))^-1);
diff(y,x);
