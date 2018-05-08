function [val]=fourier_function(z,four)


n = 2*four.N+1;

for j = 1:four.dof
for t=1:n
temp1 = sprintf('z(%d)',(j-1)*n+t);
temp2 = sprintf('v(%d,%d)',[j, t]);
eval(strcat(temp2,'=',temp1));

end
end

% disc_num = four.tf/four.ts;
% time = linspace(0,four.tf-four.ts,disc_num);



F=zeros(four.dof*four.disc_num,four.size);

for k = 1:four.disc_num 
qi   =  zeros(four.dof,1);
qdi  =  zeros(four.dof,1);
qddi =  zeros(four.dof,1);

for j = 1:four.dof
qi(j) = v(j,n);

for i =1:four.N
    t = four.time(k);
    qi(j) = qi(j)+v(j,i)/(four.w*i)*sin(four.w*i*t)-v(j,four.N+i)/(four.w*i)*cos(four.w*i*t);
    qdi(j) = qdi(j)+v(j,i)*cos(four.w*i*t)+v(j,four.N+i)*sin(four.w*i*t);
    qddi(j) = qddi(j)-v(j,i)*(four.w*i)*sin(four.w*i*t)+v(j,four.N+i)*(four.w*i)*cos(four.w*i*t);    
end
end

F((k-1)*four.dof+1:(k-1)*four.dof+four.dof,:) = four.fun(qi(1),qi(2),qi(3),qdi(1),qdi(2),qdi(3),qddi(1),qddi(2),qddi(3)).*(1./four.scale_noise.');

end

%val = cond(mpower(four.cov,-0.5)*F);

val = -log*det()


end