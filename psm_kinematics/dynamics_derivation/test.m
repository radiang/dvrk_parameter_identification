
close all
x = -10:0.1:10;
y = 2*sigmf(x,[14 0])-1;
plot(x,y)
xlabel('sigmf, P = [2 4]')
ylim([-1.05 1.05])
xlim([-2 2])