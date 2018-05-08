function [qi, qdi, qddi] = fourier_f(four)

for k = 1:length(four.time)
for j = 1:four.dof
qi(j,k) = four.v(j,2*four.N+1);
qdi(j,k) = [0];
qddi(j,k) = [0];
for i =1:four.N
    t = four.time(k);
    qi(j,k) = qi(j,k)+four.v(j,i)/(four.w*i)*sin(four.w*i*t)-four.v(j,four.N+i)/(four.w*i)*cos(four.w*i*t);
    qdi(j,k) = qdi(j,k)+four.v(j,i)*cos(four.w*i*t)+four.v(j,four.N+i)*sin(four.w*i*t);
    qddi(j,k) = qddi(j,k)-four.v(j,i)*(four.w*i)*sin(four.w*i*t)+four.v(j,four.N+i)*(four.w*i)*cos(four.w*i*t);    
end
end
end

end