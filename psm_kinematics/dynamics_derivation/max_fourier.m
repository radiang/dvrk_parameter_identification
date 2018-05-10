function [c,ceq]=max_fourier(z,four,limit,limit_vel)

n = 2*four.N+1;

 b = zeros(1,four.dof*2);
 c = zeros(1,four.dof*4);
 
for j =1:four.dof
    
    temp1 = sprintf('z(%d)',j*n);
    temp2 = sprintf('c(%d)',j);
    eval(strcat(temp2,'=',temp1));
    
    temp1 = sprintf('z(%d)',j*n);
    temp2 = sprintf('c(%d)',j+four.dof);
    eval(strcat(temp2,'=',temp1));
    
   
    
for i = 1:four.N
    
temp1 = sprintf('sqrt(z(%d)^2+z(%d)^2)/(%d*%d)',[(j-1)*n+i,(j-1)*n+four.N+i,four.w,i]);
temp2 = sprintf('arr');
eval(strcat(temp2,'=',temp1));
b(j) = b(j)+arr;

temp1 = sprintf('sqrt(z(%d)^2+z(%d)^2)',[(j-1)*n+i,(j-1)*n+four.N+i]);
temp2 = sprintf('arr2');
eval(strcat(temp2,'=',temp1));
b(j+four.dof) = b(j+four.dof)+arr2;


end

c(j) = b(j)+c(j)-limit(j);
c(2*four.dof+j) = b(j)-c(j) - limit(j);

c(j+four.dof) = b(j+four.dof)+c(j+four.dof)-limit_vel(j);
c(2*four.dof+j+four.dof) = b(j+four.dof)-c(j+four.dof)-limit_vel(j);

if (j==3)
  c(2*four.dof+j) = b(j)-c(j);   
  c(2*four.dof+j+four.dof) = b(j+four.dof)-c(j+four.dof);
end


end


ceq=[];

end