function [c,ceq]=max_fourier(z,four,limit,limit_vel)

n = 2*four.N+1;

for j =1:four.dof
    
    temp1 = sprintf('z(%d)',[n]);
    temp2 = sprintf('c(%d)',j);
    eval(strcat(temp2,'=',temp1));
    
    temp1 = sprintf('z(%d)',[n]);
    temp2 = sprintf('c(%d)',j+four.dof);
    eval(strcat(temp2,'=',temp1));
    
for i = 1:four.N
    
temp1 = sprintf('sqrt(z(%d)^2+z(%d)^2)/(%d*%d)',[(j-1)*n+i,(j-1)*n+four.N+i,four.w,i]);
temp2 = sprintf('arr(%d,%d)',[j,i]);
eval(strcat(temp2,'=',temp1));
c(j) = c(j)+arr(j,i);

temp1 = sprintf('sqrt(z(%d)^2+z(%d)^2)',[(j-1)*n+i,(j-1)*n+four.N+i]);
temp2 = sprintf('arr2(%d,%d)',[j,i]);
eval(strcat(temp2,'=',temp1));
c(j+four.dof) = c(j+four.dof)+arr2(j,i);


end

c(j) = abs(c(j))-limit(j);
c(j+four.dof) = abs(c(j+four.dof))-limit_vel(j);

end



ceq=[];

end