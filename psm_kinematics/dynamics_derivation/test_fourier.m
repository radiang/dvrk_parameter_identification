
for j= 1 :3
    a(j) = fs.vars(j*11);
    c(j) = fs.vars(j*11);
for i = 1:5
b= sqrt(fs.vars((j-1)*11+i)^2+fs.vars((j-1)*11+5+i)^2);

a(j) = a(j)+b;
c(j) = c(j)-b;

end

end